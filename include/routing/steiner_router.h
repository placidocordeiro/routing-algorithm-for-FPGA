#pragma once

#include "routing/geometry_utils.h"
#include "routing/path_finder.h"
#include "clustered_netlist.h"
#include "clustered_netlist_fwd.h"
#include "vpr_context.h"
#include "netlist_fwd.h"
#include "rr_graph_obj.h"
#include <vector>

namespace routing {

struct NetRoutingResult {
    bool success;
    int sinks_routed;
    int sinks_total;
    std::vector<std::vector<RREdgeId>> sink_paths;
};

struct RoutingResult {
    int total_nets;
    int nets_routed;
    int congestion_violations;
    std::vector<NetRoutingResult> net_results;

    void print_summary() const;
};

class SteinerRouter {
public:
    SteinerRouter() = default;

    RoutingResult route(const ClusteredNetlist& netlist, const RRGraphView& rr_graph);

private:
    // Espelha o caminho para route_ctx do VTR: escreve prev_edge, constrói
    // RouteTree e incrementa occ nos nós do novo ramo.
    void mirror_path_to_vtr(ParentNetId pnet_id, int sink_pin_index, const PathResult& path);

    NetRoutingResult route_net(
        ClusterNetId net_id,
        const ClusteredNetlist& netlist,
        const RRGraphView& rr_graph);
};

} // namespace routing
