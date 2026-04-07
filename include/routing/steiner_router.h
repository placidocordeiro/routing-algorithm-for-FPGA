#pragma once

#include "routing/geometry_utils.h"
#include "routing/path_finder.h"
#include "clustered_netlist.h"
#include "clustered_netlist_fwd.h"
#include "vpr_context.h"
#include "netlist_fwd.h"
#include "rr_graph_obj.h"
#include <vector>
#include <unordered_set>

namespace routing
{
    // Resultado de roteamento para uma net
    struct NetRoutingResult
    {
        bool success;
        int sinks_routed;
        int sinks_total;
        std::vector<PathResult> sink_paths; // Caminho completo (nós + arestas) para cada sink roteado
    };

    // Resultado geral de roteamento
    struct RoutingResult
    {
        int total_nets;
        int nets_routed;
        int congestion_violations;
        std::vector<NetRoutingResult> net_results;

        void print_summary() const;
    };

    // Router principal usando Steiner Tree
    class SteinerRouter
    {
    public:
        explicit SteinerRouter(float congestion_weight = 1.0f);

        // Roteia toda a netlist e popula g_vpr_ctx.routing() com RouteTrees e ocupação
        RoutingResult route(const ClusteredNetlist &netlist, const RRGraphView &rr_graph);

    private:
        float congestion_weight_;
        CongestionMap congestion_map_;

        // Conjunto de nós RR já comprometidos por nets anteriores.
        // Nenhuma net nova pode passar por esses nós (exclusão de recursos entre nets).
        std::unordered_set<RRNodeId> occupied_nodes_;

        // Encontra nó RRGraph mais próximo de um ponto (usado para o ponto de Steiner)
        RRNodeId find_nearest_rr_node(
            const Point &location,
            const RRGraphView &rr_graph,
            e_rr_type preferred_type) const;

        // Routing de uma rede única — também cria RouteTree no contexto VTR
        NetRoutingResult route_net(
            ClusterNetId net_id,
            const ClusteredNetlist &netlist,
            const RRGraphView &rr_graph);
    };
} // namespace routing