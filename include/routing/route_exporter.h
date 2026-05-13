#pragma once

#include "clustered_netlist.h"
#include <string>

namespace routing {

class RouteExporter {
public:
    void export_route(const std::string& place_file,
                      const std::string& route_file,
                      const ClusteredNetlist& netlist);
};

} // namespace routing
