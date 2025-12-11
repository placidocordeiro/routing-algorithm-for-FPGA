#ifndef NETLIST_TYPES_H
#define NETLIST_TYPES_H

#include <string>
#include <vector>

struct Net {
    int id;
    std::string name;
    int driver;
    std::vector<int> sinks;
};

#endif 