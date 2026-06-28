#pragma once

#include "rr_graph_view.h"
#include "netlist_fwd.h"
#include <vector>

namespace routing {

struct PathResult {
    bool success;
    std::vector<RREdgeId> edges;   // [source -> sink]
    std::vector<RRNodeId> nodes;   // [source, ..., sink]
    float total_cost;

    PathResult() : success(false), total_cost(0) {}
};

// Espelha o caminho para route_ctx do VTR: escreve prev_edge, constrói
// RouteTree e incrementa occ nos nós do novo ramo.
void mirror_path_to_vtr(ParentNetId pnet_id, int sink_pin_index, const PathResult& path);

} // namespace routing
