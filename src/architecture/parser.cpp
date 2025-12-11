#include "./parser.h"
#include "tinyxml2.h"
#include <functional>
using namespace tinyxml2;

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
