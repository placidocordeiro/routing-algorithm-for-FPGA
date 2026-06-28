#include "routing/route_bridge.h"
#include "vpr_context.h"
#include "route_tree.h"
#include "route_common.h"

namespace routing {

void mirror_path_to_vtr(
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
    hnode.index     = path.nodes.back();
    hnode.prev_edge = path.edges.back();

    auto [new_branch_root, sink_rt_node] =
        route_ctx.route_trees[pnet_id]->update_from_heap(
            &hnode, sink_pin_index, nullptr, /*is_flat=*/false);

    // Incrementar occ apenas no novo ramo — evita dupla contagem de nós
    // já presentes de sinks anteriores da mesma net.
    if (new_branch_root)
        pathfinder_update_cost_from_route_tree(*new_branch_root, +1);
}

} // namespace routing
