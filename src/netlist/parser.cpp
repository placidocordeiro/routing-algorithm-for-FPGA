#include "./parser.h"
#include "tinyxml2.h"
#include <map>
#include <sstream>
using namespace tinyxml2;

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
