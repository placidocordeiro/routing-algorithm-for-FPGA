#include "routing/route_exporter.h"

#include "netlist.h"
#include "read_route.h"

namespace routing {

void RouteExporter::export_route(const std::string& place_file,
                                 const std::string& route_file,
                                 const ClusteredNetlist& netlist) {
    ::print_route((const Netlist<>&)netlist,
                  place_file.c_str(),
                  route_file.c_str(),
                  /*is_flat=*/false);
}

} // namespace routing
