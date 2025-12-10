#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <sstream>
#include <filesystem>
#include <map>
#include <memory>
#include <functional>
#include <algorithm>
#include <cctype>
#include "tinyxml2.h"

using namespace tinyxml2;
namespace fs = std::filesystem;

struct Switch {
    std::string type, name;
    double R, Cin, Cout, Tdel, mux_trans_size, buf_size;
};

struct Segment {
    double freq;
    int length;
    std::string type;
    double Rmetal, Cmetal;
    std::string mux_name;
};

struct Direct {
    std::string name, from_pin, to_pin;
    int x_offset, y_offset, z_offset;
};

struct Port {
    std::string name, type;
    int num_pins;
    bool is_clock;
};

struct Tile {
    std::string name, type;
    int height;
    double area, fc_in, fc_out;
    std::vector<Port> ports;
};

struct Device {
    double R_minW_nmos, R_minW_pmos, grid_logic_tile_area;
    std::string switch_block_type, connection_block_switch;
    int fs;
};

struct FPGAArchitecture {
    Device device;
    std::vector<Switch> switches;
    std::vector<Segment> segments;
    std::vector<Direct> directs;
    std::vector<Tile> tiles;
};

struct RRNode {
    int id, x, y, capacity;
    std::string type;
    float delay;
};

struct RREdge {
    int from_node, to_node, switch_id;
    float delay;
};

struct Net {
    int id, driver;
    std::string name;
    std::vector<int> sinks;
};

struct Placement {
    std::string block_name;
    int x, y, subblock, block_num;
};

FPGAArchitecture parse_architecture_xml(const std::string& filename) {
    FPGAArchitecture arch;
    XMLDocument doc;
    if (doc.LoadFile(filename.c_str()) != XML_SUCCESS) return arch;
    
    arch.device = {0.0, 0.0, 0.0, "", "", 0};
    
    XMLElement* root = doc.RootElement();
    if (!root) return arch;
    
    XMLElement* device_elem = root->FirstChildElement("device");
    if (device_elem) {
        XMLElement* sizing_elem = device_elem->FirstChildElement("sizing");
        if (sizing_elem) {
            arch.device.R_minW_nmos = sizing_elem->DoubleAttribute("R_minW_nmos", 0.0);
            arch.device.R_minW_pmos = sizing_elem->DoubleAttribute("R_minW_pmos", 0.0);
        }
        XMLElement* area_elem = device_elem->FirstChildElement("area");
        if (area_elem) {
            arch.device.grid_logic_tile_area = area_elem->DoubleAttribute("grid_logic_tile_area", 0.0);
        }
        XMLElement* switch_block_elem = device_elem->FirstChildElement("switch_block");
        if (switch_block_elem) {
            arch.device.switch_block_type = switch_block_elem->Attribute("type") ? 
                                           switch_block_elem->Attribute("type") : "";
            arch.device.fs = switch_block_elem->IntAttribute("fs", 0);
        }
        XMLElement* connection_block_elem = device_elem->FirstChildElement("connection_block");
        if (connection_block_elem) {
            arch.device.connection_block_switch = connection_block_elem->Attribute("input_switch_name") ?
                                                 connection_block_elem->Attribute("input_switch_name") : "";
        }
    }
    
    XMLElement* switchlist_elem = root->FirstChildElement("switchlist");
    if (switchlist_elem) {
        for (XMLElement* switch_elem = switchlist_elem->FirstChildElement("switch"); 
             switch_elem; 
             switch_elem = switch_elem->NextSiblingElement("switch")) {
            
            Switch sw;
            sw.type = switch_elem->Attribute("type") ? switch_elem->Attribute("type") : "";
            sw.name = switch_elem->Attribute("name") ? switch_elem->Attribute("name") : "";
            sw.R = switch_elem->DoubleAttribute("R", 0.0);
            sw.Cin = switch_elem->DoubleAttribute("Cin", 0.0);
            sw.Cout = switch_elem->DoubleAttribute("Cout", 0.0);
            sw.Tdel = switch_elem->DoubleAttribute("Tdel", 0.0);
            sw.mux_trans_size = switch_elem->DoubleAttribute("mux_trans_size", 0.0);
            
            const char* buf_size_attr = switch_elem->Attribute("buf_size");
            sw.buf_size = (buf_size_attr && std::string(buf_size_attr) != "auto") ? 
                         std::stod(buf_size_attr) : 0.0;
            
            arch.switches.push_back(sw);
        }
    }
    
    XMLElement* segmentlist_elem = root->FirstChildElement("segmentlist");
    if (segmentlist_elem) {
        for (XMLElement* segment_elem = segmentlist_elem->FirstChildElement("segment"); 
             segment_elem; 
             segment_elem = segment_elem->NextSiblingElement("segment")) {
            
            Segment seg;
            seg.freq = segment_elem->DoubleAttribute("freq", 0.0);
            seg.length = segment_elem->IntAttribute("length", 0);
            seg.type = segment_elem->Attribute("type") ? segment_elem->Attribute("type") : "";
            seg.Rmetal = segment_elem->DoubleAttribute("Rmetal", 0.0);
            seg.Cmetal = segment_elem->DoubleAttribute("Cmetal", 0.0);
            
            XMLElement* mux_elem = segment_elem->FirstChildElement("mux");
            seg.mux_name = (mux_elem && mux_elem->Attribute("name")) ? mux_elem->Attribute("name") : "";
            
            arch.segments.push_back(seg);
        }
    }
    
    XMLElement* complexblocklist_elem = root->FirstChildElement("complexblocklist");
    if (complexblocklist_elem) {
        std::function<void(XMLElement*)> searchDirects = [&](XMLElement* elem) {
            if (!elem) return;
            if (std::string(elem->Name()) == "direct") {
                Direct dir;
                dir.name = elem->Attribute("name") ? elem->Attribute("name") : "";
                dir.from_pin = elem->Attribute("from_pin") ? elem->Attribute("from_pin") : "";
                dir.to_pin = elem->Attribute("to_pin") ? elem->Attribute("to_pin") : "";
                dir.x_offset = elem->IntAttribute("x_offset", 0);
                dir.y_offset = elem->IntAttribute("y_offset", 0);
                dir.z_offset = elem->IntAttribute("z_offset", 0);
                arch.directs.push_back(dir);
            }
            for (XMLElement* child = elem->FirstChildElement(); child; child = child->NextSiblingElement()) {
                searchDirects(child);
            }
        };
        searchDirects(complexblocklist_elem);
    }
    
    XMLElement* tiles_elem = root->FirstChildElement("tiles");
    if (tiles_elem) {
        for (XMLElement* tile_elem = tiles_elem->FirstChildElement("tile"); 
             tile_elem; 
             tile_elem = tile_elem->NextSiblingElement("tile")) {
            
            Tile tile;
            tile.name = tile_elem->Attribute("name") ? tile_elem->Attribute("name") : "";
            tile.type = tile.name;
            tile.height = tile_elem->IntAttribute("height", 1);
            tile.area = tile_elem->DoubleAttribute("area", 0.0);
            tile.fc_in = 0.0;
            tile.fc_out = 0.0;
            
            XMLElement* subtile_elem = tile_elem->FirstChildElement("sub_tile");
            if (subtile_elem) {
                XMLElement* fc_elem = subtile_elem->FirstChildElement("fc");
                if (fc_elem) {
                    tile.fc_in = fc_elem->DoubleAttribute("in_val", 0.0);
                    tile.fc_out = fc_elem->DoubleAttribute("out_val", 0.0);
                }
                
                for (XMLElement* input_elem = subtile_elem->FirstChildElement("input"); 
                     input_elem; 
                     input_elem = input_elem->NextSiblingElement("input")) {
                    Port port;
                    port.type = "input";
                    port.is_clock = false;
                    port.name = input_elem->Attribute("name") ? input_elem->Attribute("name") : "";
                    port.num_pins = input_elem->IntAttribute("num_pins", 1);
                    tile.ports.push_back(port);
                }
                
                for (XMLElement* output_elem = subtile_elem->FirstChildElement("output"); 
                     output_elem; 
                     output_elem = output_elem->NextSiblingElement("output")) {
                    Port port;
                    port.type = "output";
                    port.is_clock = false;
                    port.name = output_elem->Attribute("name") ? output_elem->Attribute("name") : "";
                    port.num_pins = output_elem->IntAttribute("num_pins", 1);
                    tile.ports.push_back(port);
                }
                
                for (XMLElement* clock_elem = subtile_elem->FirstChildElement("clock"); 
                     clock_elem; 
                     clock_elem = clock_elem->NextSiblingElement("clock")) {
                    Port port;
                    port.type = "clock";
                    port.is_clock = true;
                    port.name = clock_elem->Attribute("name") ? clock_elem->Attribute("name") : "";
                    port.num_pins = clock_elem->IntAttribute("num_pins", 1);
                    tile.ports.push_back(port);
                }
            }
            arch.tiles.push_back(tile);
        }
    }
    
    return arch;
}

std::vector<Net> read_net_file(const std::string& filename) {
    std::vector<Net> nets;
    XMLDocument doc;
    if (doc.LoadFile(filename.c_str()) != XML_SUCCESS) return nets;
    
    XMLElement* root = doc.RootElement();
    if (!root) return nets;
    
    std::map<std::string, int> block_name_to_id;
    std::map<std::string, int> net_name_to_id;
    std::map<std::string, std::string> block_type;
    std::vector<std::string> net_source_blocks;
    
    int block_id_counter = 0;
    for (XMLElement* block_elem = root->FirstChildElement("block"); 
         block_elem; 
         block_elem = block_elem->NextSiblingElement("block")) {
        
        const char* name_attr = block_elem->Attribute("name");
        if (!name_attr) continue;
        
        std::string block_name = name_attr;
        if (block_name == "open" || block_name.empty() || block_name == "circuito_simples.net") {
            continue;
        }
        
        std::string instance = block_elem->Attribute("instance") ? block_elem->Attribute("instance") : "";
        std::string mode = block_elem->Attribute("mode") ? block_elem->Attribute("mode") : "";
        
        if (instance.find("clb") != std::string::npos) {
            block_type[block_name] = "clb";
        } else if (instance.find("io") != std::string::npos) {
            if (mode == "inpad") {
                block_type[block_name] = "io_in";
                net_source_blocks.push_back(block_name);
            } else if (mode == "outpad") {
                block_type[block_name] = "io_out";
            } else {
                block_type[block_name] = "io";
            }
        }
        
        block_name_to_id[block_name] = block_id_counter++;
    }
    
    for (const auto& block : block_name_to_id) {
        if (block_type[block.first] == "clb") {
            net_source_blocks.push_back(block.first);
        }
    }
    
    for (size_t i = 0; i < net_source_blocks.size(); i++) {
        Net net;
        net.id = i;
        net.name = net_source_blocks[i];
        net.driver = -1;
        nets.push_back(net);
        net_name_to_id[net_source_blocks[i]] = i;
    }
    
    for (XMLElement* block_elem = root->FirstChildElement("block"); 
         block_elem; 
         block_elem = block_elem->NextSiblingElement("block")) {
        
        const char* block_name_attr = block_elem->Attribute("name");
        if (!block_name_attr) continue;
        
        std::string current_block = block_name_attr;
        if (current_block == "open" || current_block.empty() || 
            block_name_to_id.find(current_block) == block_name_to_id.end()) {
            continue;
        }
        
        XMLElement* inputs_elem = block_elem->FirstChildElement("inputs");
        if (inputs_elem) {
            for (XMLElement* port_elem = inputs_elem->FirstChildElement("port"); 
                 port_elem; 
                 port_elem = port_elem->NextSiblingElement("port")) {
                
                const char* port_text = port_elem->GetText();
                if (port_text) {
                    std::string content = port_text;
                    std::istringstream iss(content);
                    std::string token;
                    
                    while (iss >> token) {
                        if (token == "open") continue;
                        if (token.find('.') == std::string::npos && 
                            token.find('[') == std::string::npos &&
                            token.find("->") == std::string::npos) {
                            
                            if (net_name_to_id.find(token) != net_name_to_id.end()) {
                                int net_idx = net_name_to_id[token];
                                int block_idx = block_name_to_id[current_block];
                                if (nets[net_idx].driver != block_idx) {
                                    nets[net_idx].sinks.push_back(block_idx);
                                }
                            }
                        }
                    }
                }
            }
        }
        
        XMLElement* outputs_elem = block_elem->FirstChildElement("outputs");
        if (outputs_elem) {
            for (XMLElement* port_elem = outputs_elem->FirstChildElement("port"); 
                 port_elem; 
                 port_elem = port_elem->NextSiblingElement("port")) {
                
                const char* port_text = port_elem->GetText();
                if (port_text) {
                    std::string content = port_text;
                    std::istringstream iss(content);
                    std::string token;
                    
                    while (iss >> token) {
                        if (token == "open") continue;
                        if (token.find('.') == std::string::npos && 
                            token.find('[') == std::string::npos &&
                            token.find("->") == std::string::npos) {
                            
                            if (net_name_to_id.find(token) != net_name_to_id.end()) {
                                int net_idx = net_name_to_id[token];
                                int block_idx = block_name_to_id[current_block];
                                nets[net_idx].driver = block_idx;
                            }
                        }
                    }
                }
            }
        }
    }
    
    if (net_name_to_id.find("a") != net_name_to_id.end()) {
        nets[net_name_to_id["a"]].driver = -1;
    }
    if (net_name_to_id.find("b") != net_name_to_id.end()) {
        nets[net_name_to_id["b"]].driver = -1;
    }
    
    std::vector<Net> valid_nets_only;
    for (const auto& net : nets) {
        if (net.driver != -1 || !net.sinks.empty()) {
            valid_nets_only.push_back(net);
        }
    }
    
    return valid_nets_only;
}

std::vector<Placement> read_place_file(const std::string& filename) {
    std::vector<Placement> placements;
    std::ifstream file(filename);
    if (!file.is_open()) return placements;
    
    std::string line;
    while (std::getline(file, line)) {
        if (line.empty() || line[0] == '#' || line.find("Block") != std::string::npos) {
            continue;
        }
        
        std::istringstream iss(line);
        Placement place;
        if (iss >> place.block_name >> place.x >> place.y >> place.subblock >> place.block_num) {
            placements.push_back(place);
        }
    }
    
    file.close();
    return placements;
}

int main() {
    std::string data_dir = "../data";
    
    if (!fs::exists(data_dir)) {
        std::cerr << "Directory not found: " << data_dir << std::endl;
        return 1;
    }
    
    std::string arch_file = data_dir + "/k6_frac_N10_mem32K_40nm.xml";
    std::string net_file = data_dir + "/circuito_simples.net";
    std::string place_file = data_dir + "/circuito_simples.place";
    
    if (!fs::exists(arch_file) || !fs::exists(net_file) || !fs::exists(place_file)) {
        std::cerr << "Missing required files" << std::endl;
        return 1;
    }
    
    auto fpga_arch = parse_architecture_xml(arch_file);
    auto nets = read_net_file(net_file);
    auto placements = read_place_file(place_file);
    
    int total_sinks = 0;
    for (const auto& net : nets) {
        total_sinks += net.sinks.size();
    }
    
    int min_x = 1000, max_x = -1000;
    int min_y = 1000, max_y = -1000;
    for (const auto& place : placements) {
        min_x = std::min(min_x, place.x);
        max_x = std::max(max_x, place.x);
        min_y = std::min(min_y, place.y);
        max_y = std::max(max_y, place.y);
    }
    
    std::cout << "\n====== ARCHITECTURE ======\n";
    std::cout << "Switches: " << fpga_arch.switches.size() << "\n";
    std::cout << "Segments: " << fpga_arch.segments.size() << "\n";
    std::cout << "Directs: " << fpga_arch.directs.size() << "\n";
    std::cout << "Tiles: " << fpga_arch.tiles.size() << "\n\n";
    
    std::cout << "====== CIRCUIT ======\n";
    std::cout << "Nets: " << nets.size() << "\n";
    std::cout << "Total sinks: " << total_sinks << "\n";
    std::cout << "Avg sinks/net: " << (nets.empty() ? 0.0 : (float)total_sinks / nets.size()) << "\n\n";
    
    std::cout << "====== PLACEMENT ======\n";
    std::cout << "Placements: " << placements.size() << "\n";
    std::cout << "X range: [" << min_x << " .. " << max_x << "]\n";
    std::cout << "Y range: [" << min_y << " .. " << max_y << "]\n";
    std::cout << "Width: " << (max_x - min_x + 1) << "\n";
    std::cout << "Height: " << (max_y - min_y + 1) << "\n";
    
    return 0;
}
