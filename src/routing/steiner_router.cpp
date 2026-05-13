#include "routing/steiner_router.h"
#include "vpr_context.h"
#include "vpr_api.h"
#include "route_tree.h"
#include "route_common.h"
#include <iostream>

namespace routing {

void RoutingResult::print_summary() const {
    std::cout << "\n=== Routing Summary ===\n";
    std::cout << "Total nets: " << total_nets << "\n";
    std::cout << "Nets routed: " << nets_routed << "\n";
    if (total_nets > 0) {
        std::cout << "Success rate: " << (100.0f * nets_routed / total_nets) << "%\n";
    }
    std::cout << "Congestion violations: " << congestion_violations << "\n";
}

SteinerRouter::SteinerRouter(float congestion_weight)
    : congestion_weight_(congestion_weight) {}

Point SteinerRouter::get_block_location(
    ClusterBlockId block_id,
    const ClusteredNetlist& /*netlist*/) const {
    auto& place_ctx = g_vpr_ctx.placement();
    const auto& block_loc = place_ctx.block_locs()[block_id];
    return Point(static_cast<float>(block_loc.loc.x), static_cast<float>(block_loc.loc.y));
}

void SteinerRouter::mirror_path_to_vtr(
    ParentNetId pnet_id,
    int sink_pin_index,
    const PathResult& path) {

    if (!path.success || path.nodes.empty() || path.edges.empty()) {
        return;
    }

    auto& route_ctx = g_vpr_ctx.mutable_routing();

    // 1) Escrever prev_edge em cada nó do caminho.
    //    nodes = [source, n1, n2, ..., sink]
    //    edges = [e0,  e1, ..., e_{k-1}]
    //    e_i é a aresta de nodes[i] -> nodes[i+1], então prev_edge de
    //    nodes[i+1] é edges[i]. update_from_heap usará isso no walkback.
    for (size_t i = 0; i + 1 < path.nodes.size(); ++i) {
        route_ctx.rr_node_route_inf[path.nodes[i + 1]].prev_edge = path.edges[i];
    }

    // 2) Garantir RouteTree existe (criada na 1a vez, com SOURCE como raiz).
    if (!route_ctx.route_trees[pnet_id]) {
        route_ctx.route_trees[pnet_id].emplace(pnet_id);
    }

    // 3) Construir RTExploredNode apontando para o sink.
    RTExploredNode hnode;
    hnode.index = path.nodes.back();
    hnode.prev_edge = path.edges.back();

    // 4) Chamar update_from_heap — walkback usando prev_edge.
    auto [new_branch_root, sink_rt_node] =
        route_ctx.route_trees[pnet_id]->update_from_heap(
            &hnode,
            sink_pin_index,
            /*spatial_rt_lookup=*/nullptr,
            /*is_flat=*/false);

    // 5) Atualizar ocupância (occ +=1) apenas no novo ramo, evitando
    //    dupla contagem de nós já presentes da rota de sinks anteriores.
    if (new_branch_root) {
        pathfinder_update_cost_from_route_tree(*new_branch_root, +1);
    }
}

NetRoutingResult SteinerRouter::route_net(
    ClusterNetId net_id,
    const ClusteredNetlist& netlist,
    const RRGraphView& /*rr_graph*/) {

    NetRoutingResult result;
    result.success = false;
    result.sinks_total = netlist.net_sinks(net_id).size();
    result.sinks_routed = 0;

    auto& route_ctx = g_vpr_ctx.routing();
    ParentNetId pnet_id{size_t(net_id)};

    const auto& terminals = route_ctx.net_rr_terminals[pnet_id];
    if (terminals.size() < 2) {
        // Sem sinks (net global ou apenas driver) — nada a rotear.
        return result;
    }

    RRNodeId source_node = terminals[0];
    if (!source_node.is_valid()) {
        std::cerr << "Aviso: SOURCE inválido para net " << size_t(net_id) << "\n";
        return result;
    }

    // Rotear source -> cada sink via PathFinder (Dijkstra com congestionamento).
    PathFinder pf(g_vpr_ctx.device().rr_graph, congestion_map_);

    for (size_t i = 1; i < terminals.size(); ++i) {
        RRNodeId sink_node = terminals[i];
        if (!sink_node.is_valid()) {
            std::cerr << "Aviso: SINK inválido para net " << size_t(net_id)
                      << " pin " << i << "\n";
            continue;
        }

        PathFindingParams params;
        params.source = source_node;
        params.sink = sink_node;
        params.congestion_weight = congestion_weight_;

        PathResult path = pf.find_path(params);

        if (path.success) {
            result.sink_paths.push_back(path.edges);
            result.sinks_routed++;
            pf.commit_path(path);
            mirror_path_to_vtr(pnet_id, static_cast<int>(i), path);
        }
    }

    result.success = (result.sinks_routed == result.sinks_total);
    return result;
}

RoutingResult SteinerRouter::route(
    const ClusteredNetlist& netlist,
    const RRGraphView& rr_graph) {

    RoutingResult result;
    result.total_nets = netlist.nets().size();
    result.nets_routed = 0;
    result.congestion_violations = 0;

    std::cout << "Starting routing of " << result.total_nets << " nets...\n";

    for (ClusterNetId net_id : netlist.nets()) {
        NetRoutingResult net_result = route_net(net_id, netlist, rr_graph);

        if (net_result.success) {
            result.nets_routed++;
        }

        result.net_results.push_back(net_result);

        if ((result.net_results.size() % 100) == 0) {
            std::cout << "Routed " << result.net_results.size() << "/" << result.total_nets << " nets\n";
        }
    }

    // Contar violations reais (occ > capacity) — mesma métrica que feasible_routing().
    auto& route_ctx = g_vpr_ctx.routing();
    for (RRNodeId rr_id : rr_graph.nodes()) {
        int occ = route_ctx.rr_node_route_inf[rr_id].occ();
        int cap = rr_graph.node_capacity(rr_id);
        if (occ > cap) result.congestion_violations++;
    }

    return result;
}

} // namespace routing
