#include "routing/path_finder.h"
#include "globals.h"
#include <algorithm>
#include <queue>
#include <map>

namespace routing {

PathResult PathFinder::find_path(const PathFindingParams& params) {
    PathResult result;

    auto& route_ctx = g_vpr_ctx.routing();

    std::priority_queue<ExploredNode, std::vector<ExploredNode>, std::greater<ExploredNode>> heap;
    std::map<RRNodeId, float> best_cost;
    std::map<RRNodeId, RREdgeId> parent_edge;
    std::map<RRNodeId, RRNodeId> parent_node;

    heap.push({params.source, 0.0f, RREdgeId::INVALID()});
    best_cost[params.source] = 0.0f;

    int iterations = 0;

    while (!heap.empty() && iterations < params.max_iterations) {
        ++iterations;

        auto current = heap.top();
        heap.pop();

        if (current.node_id == params.sink) {
            result.success = true;
            result.total_cost = current.cost;

            // Backtrace: nodes = [source, ..., sink], edges = [e0, ..., e_{k-1}]
            RRNodeId node = params.sink;
            result.nodes.push_back(node);
            while (parent_edge.count(node) && parent_edge[node].is_valid()) {
                result.edges.push_back(parent_edge[node]);
                node = parent_node[node];
                result.nodes.push_back(node);
            }
            std::reverse(result.edges.begin(), result.edges.end());
            std::reverse(result.nodes.begin(), result.nodes.end());
            return result;
        }

        if (best_cost.count(current.node_id) &&
            best_cost[current.node_id] < current.cost) {
            continue;
        }

        RREdgeId first_edge = rr_graph_.node_first_edge(current.node_id);
        RREdgeId last_edge  = rr_graph_.node_last_edge(current.node_id);
        size_t num_edges = static_cast<size_t>(last_edge) - static_cast<size_t>(first_edge);

        for (size_t edge_idx = 0; edge_idx < num_edges; ++edge_idx) {
            RREdgeId out_edge   = RREdgeId(static_cast<size_t>(first_edge) + edge_idx);
            RRNodeId next_node  = rr_graph_.edge_sink_node(current.node_id, edge_idx);
            if (!next_node.is_valid()) continue;

            // Hard capacity: descartar vizinhos já saturados, exceto o sink alvo.
            if (next_node != params.sink) {
                int occ = route_ctx.rr_node_route_inf[next_node].occ();
                int cap = rr_graph_.node_capacity(next_node);
                if (occ >= cap) continue;
            }

            float new_cost = current.cost + 1.0f;  // custo uniforme por aresta

            if (!best_cost.count(next_node) || best_cost[next_node] > new_cost) {
                best_cost[next_node]  = new_cost;
                parent_edge[next_node] = out_edge;
                parent_node[next_node] = current.node_id;
                heap.push({next_node, new_cost, out_edge});
            }
        }
    }

    result.success = false;
    return result;
}

} // namespace routing
