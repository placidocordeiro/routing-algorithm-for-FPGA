#pragma once

#include "routing/geometry_utils.h"
#include "routing/path_finder.h"
#include "clustered_netlist.h"
#include "clustered_netlist_fwd.h"
#include "vpr_context.h"
#include "netlist_fwd.h"
#include "rr_graph_obj.h"
#include <map>
#include <tuple>
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

    // Índice espacial: (x, y, ptc, type) -> RRNodeId
    // Construído uma vez no início do roteamento para evitar busca linear
    std::map<std::tuple<int, int, int, e_rr_type>, RRNodeId> rr_node_index_;

    // Constrói o índice espacial a partir do RR graph
    void build_rr_node_index(const RRGraphView& rr_graph);

    // Mapeia um pin físico para seu nó RR correspondente (OPIN para driver, IPIN para sink)
    RRNodeId find_rr_node_for_pin(
        ClusterPinId pin_id,
        const ClusteredNetlist& netlist,
        const RRGraphView& rr_graph,
        e_rr_type expected_type) const;

    // Obtém localização de um bloco no espaço 2D
    Point get_block_location(ClusterBlockId block_id, const ClusteredNetlist& netlist) const;

    // Routing de uma rede única
    NetRoutingResult route_net(
        ClusterNetId net_id,
        const ClusteredNetlist& netlist,
        const RRGraphView& rr_graph);

    // Encontra nó RRGraph mais próximo de um ponto (usado para o ponto de Steiner)
    RRNodeId find_nearest_rr_node(
        const Point& location,
        const RRGraphView& rr_graph,
        e_rr_type preferred_type) const;
};

} // namespace routing