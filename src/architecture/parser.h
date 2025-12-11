#ifndef ARCHITECTURE_PARSER_H
#define ARCHITECTURE_PARSER_H

#include "architecture/types.h"
#include <string>

FPGAArchitecture parse_architecture_xml(const std::string& filename);

#endif