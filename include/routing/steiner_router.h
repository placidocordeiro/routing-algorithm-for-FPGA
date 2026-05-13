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

// Resultado de roteamento para uma net
struct NetRoutingResult {
    bool success;
    int sinks_routed;
    int sinks_total;
    std::vector<std::vector<RREdgeId>> sink_paths;  // Caminho para cada sink
};

// Resultado geral de roteamento
struct RoutingResult {
    int total_nets;
    int nets_routed;
    int congestion_violations;
    std::vector<NetRoutingResult> net_results;

    void print_summary() const;
};

// Router principal usando Steiner Tree
class SteinerRouter {
public:
    explicit SteinerRouter(float congestion_weight = 1.0f);

    // Roteia toda a netlist
    RoutingResult route(const ClusteredNetlist& netlist, const RRGraphView& rr_graph);

private:
    float congestion_weight_;
    CongestionMap congestion_map_;

    // Obtém localização de um bloco no espaço 2D
    Point get_block_location(ClusterBlockId block_id, const ClusteredNetlist& netlist) const;

    // Routing de uma rede única
    NetRoutingResult route_net(
        ClusterNetId net_id,
        const ClusteredNetlist& netlist,
        const RRGraphView& rr_graph);

    // Espelha o caminho encontrado pelo nosso Dijkstra para as estruturas do VTR:
    // escreve prev_edge em rr_node_route_inf, chama RouteTree::update_from_heap
    // e atualiza a ocupância nos nós do novo ramo.
    void mirror_path_to_vtr(
        ParentNetId pnet_id,
        int sink_pin_index,
        const PathResult& path);
};

} // namespace routing
