#include "./parser.h"
#include <fstream>
#include <sstream>

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
