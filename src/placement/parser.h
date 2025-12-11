#ifndef PLACEMENT_PARSER_H
#define PLACEMENT_PARSER_H

#include "placement/types.h"
#include <vector>
#include <string>

std::vector<Placement> read_place_file(const std::string& filename);

#endif