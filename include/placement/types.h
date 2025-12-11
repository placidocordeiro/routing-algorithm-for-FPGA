#ifndef PLACEMENT_TYPES_H
#define PLACEMENT_TYPES_H

#include <string>

struct Placement {
    std::string block_name;
    int x, y, subblock, block_num;
    std::string pin_name;
};

#endif