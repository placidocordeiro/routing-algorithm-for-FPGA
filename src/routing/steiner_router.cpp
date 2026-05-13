#include "routing/steiner_router.h"
#include "vpr_context.h"
#include "route_tree.h"
#include "route_common.h"
#include <iostream>

namespace routing {

void RoutingResult::print_summary() const {
    std::cout << "\n=== Routing Summary ===\n";
    std::cout << "Total nets: " << total_nets << "\n";
    std::cout << "Nets routed: " << nets_routed << "\n";
    if (total_nets > 0)
        std::cout << "Success rate: " << (100.0f * nets_routed / total_nets) << "%\n";
    std::cout << "Congestion violations: " << congestion_violations << "\n";
}

void SteinerRouter::mirror_path_to_vtr(
    ParentNetId pnet_id,
    int sink_pin_index,
    const PathResult& path) {

    if (!path.success || path.nodes.empty() || path.edges.empty())
        return;

    auto& route_ctx = g_vpr_ctx.mutable_routing();

    // Escrever prev_edge para cada nó do caminho [source, ..., sink].
    // update_from_heap usa esse campo no walkback.
    for (size_t i = 0; i + 1 < path.nodes.size(); ++i)
        route_ctx.rr_node_route_inf[path.nodes[i + 1]].prev_edge = path.edges[i];

    if (!route_ctx.route_trees[pnet_id])
        route_ctx.route_trees[pnet_id].emplace(pnet_id);

    RTExploredNode hnode;
    hnode.index    = path.nodes.back();
    hnode.prev_edge = path.edges.back();

    auto [new_branch_root, sink_rt_node] =
        route_ctx.route_trees[pnet_id]->update_from_heap(
            &hnode, sink_pin_index, nullptr, /*is_flat=*/false);

    // Incrementar occ apenas no novo ramo — evita dupla contagem de nós
    // já presentes de sinks anteriores da mesma net.
    if (new_branch_root)
        pathfinder_update_cost_from_route_tree(*new_branch_root, +1);
}

NetRoutingResult SteinerRouter::route_net(
    ClusterNetId net_id,
    const ClusteredNetlist& netlist,
    const RRGraphView& rr_graph) {

    NetRoutingResult result;
    result.success     = false;
    result.sinks_total = netlist.net_sinks(net_id).size();
    result.sinks_routed = 0;

    auto& route_ctx = g_vpr_ctx.routing();
    ParentNetId pnet_id{size_t(net_id)};

    const auto& terminals = route_ctx.net_rr_terminals[pnet_id];
    if (terminals.size() < 2)
        return result;

    RRNodeId source_node = terminals[0];
    if (!source_node.is_valid()) {
        std::cerr << "Aviso: SOURCE inválido para net " << size_t(net_id) << "\n";
        return result;
    }

    PathFinder pf(rr_graph);

    for (size_t i = 1; i < terminals.size(); ++i) {
        RRNodeId sink_node = terminals[i];
        if (!sink_node.is_valid()) {
            std::cerr << "Aviso: SINK inválido para net " << size_t(net_id)
                      << " pin " << i << "\n";
            continue;
        }

        PathFindingParams params;
        params.source = source_node;
        params.sink   = sink_node;

        PathResult path = pf.find_path(params);

        if (path.success) {
            result.sink_paths.push_back(path.edges);
            result.sinks_routed++;
            // mirror_path_to_vtr incrementa occ nos nós do caminho,
            // tornando-os indisponíveis para nets subsequentes.
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
    result.total_nets          = netlist.nets().size();
    result.nets_routed         = 0;
    result.congestion_violations = 0;

    std::cout << "Starting routing of " << result.total_nets << " nets...\n";

    for (ClusterNetId net_id : netlist.nets()) {
        NetRoutingResult net_result = route_net(net_id, netlist, rr_graph);
        if (net_result.success)
            result.nets_routed++;
        result.net_results.push_back(net_result);

        if ((result.net_results.size() % 100) == 0)
            std::cout << "Routed " << result.net_results.size()
                      << "/" << result.total_nets << " nets\n";
    }

    // Violations por nó — mesma métrica que feasible_routing().
    auto& route_ctx = g_vpr_ctx.routing();
    for (RRNodeId rr_id : rr_graph.nodes()) {
        if (route_ctx.rr_node_route_inf[rr_id].occ() > rr_graph.node_capacity(rr_id))
            result.congestion_violations++;
    }

    return result;
}

} // namespace routing
