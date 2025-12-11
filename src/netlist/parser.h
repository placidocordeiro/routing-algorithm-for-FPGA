#ifndef NETLIST_PARSER_H
#define NETLIST_PARSER_H

#include "netlist/types.h"
#include <vector>
#include <string>

std::vector<Net> read_net_file(const std::string& filename);

#endif