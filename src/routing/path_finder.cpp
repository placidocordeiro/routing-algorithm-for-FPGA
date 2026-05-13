#include "routing/path_finder.h"
#include <algorithm>
#include <queue>
#include <map>
#include <cmath>
#include <cstddef>

namespace routing {

float PathFinder::heuristic_cost(RRNodeId current, RRNodeId target) const {
    // Simplificado: retorna 0 (Dijkstra puro)
    // Poderia usar Manhattan distance se usando as coordenadas
    return 0.0f;
}

float PathFinder::edge_cost(RREdgeId edge_id, float congestion_weight) const {
    // Base cost é fixo, podemos buscar o custo real da aresta se necessário
    float base_cost = 1.0f;
    float congestion_cost = congestion_.get_congestion_cost(edge_id);
    return base_cost + congestion_weight * congestion_cost;
}

PathResult PathFinder::find_path(const PathFindingParams& params) {
    PathResult result;

    // Priority queue: (cost, node_id)
    std::priority_queue<ExploredNode, std::vector<ExploredNode>, std::greater<ExploredNode>> heap;

    // Maps: node_id -> (cost, came_from_edge)
    std::map<RRNodeId, float> best_cost;
    std::map<RRNodeId, RREdgeId> parent_edge;
    std::map<RRNodeId, RRNodeId> parent_node;  // Para backtrace

    // Inicializar
    heap.push({params.source, 0.0f, RREdgeId::INVALID()});
    best_cost[params.source] = 0.0f;

    int iterations = 0;
    const int max_iter = params.max_iterations;

    while (!heap.empty() && iterations < max_iter) {
        iterations++;

        auto current = heap.top();
        heap.pop();

        // Se já encontramos o sink, retroceder
        if (current.node_id == params.sink) {
            result.success = true;
            result.total_cost = current.cost;

            // Reconstruir caminho fazendo backtrace.
            // edges fica ordenado [edge0, edge1, ...] (source -> sink).
            // nodes fica ordenado [source, ..., sink], com nodes.size() == edges.size() + 1.
            RRNodeId node = params.sink;
            result.nodes.push_back(node);
            while (parent_edge.count(node) > 0 && parent_edge[node].is_valid()) {
                result.edges.push_back(parent_edge[node]);
                node = parent_node[node];
                result.nodes.push_back(node);
            }
            std::reverse(result.edges.begin(), result.edges.end());
            std::reverse(result.nodes.begin(), result.nodes.end());

            return result;
        }

        // Pular se já visitamos com custo menor
        if (best_cost.count(current.node_id) > 0 &&
            best_cost[current.node_id] < current.cost) {
            continue;
        }

        // Explorar vizinhos (arestas de saída)
        RREdgeId first_edge = rr_graph_.node_first_edge(current.node_id);
        RREdgeId last_edge = rr_graph_.node_last_edge(current.node_id);

        // Iterarcalculando índice baseado em IDs
        size_t num_edges = static_cast<size_t>(last_edge) - static_cast<size_t>(first_edge);
        for (size_t edge_idx = 0; edge_idx < num_edges; ++edge_idx) {
            RREdgeId out_edge = RREdgeId(static_cast<size_t>(first_edge) + edge_idx);
            RRNodeId next_node = rr_graph_.edge_sink_node(current.node_id, edge_idx);
            if (!next_node.is_valid()) continue;

            float edge_cost_val = edge_cost(out_edge, params.congestion_weight);
            float new_cost = current.cost + edge_cost_val;

            // Atualizar se encontramos caminho melhor
            if (best_cost.count(next_node) == 0 || best_cost[next_node] > new_cost) {
                best_cost[next_node] = new_cost;
                parent_edge[next_node] = out_edge;
                parent_node[next_node] = current.node_id;

                float h = heuristic_cost(next_node, params.sink);
                heap.push({next_node, new_cost + h, out_edge});
            }
        }
    }

    // Nenhum caminho encontrado
    result.success = false;
    return result;
}

void PathFinder::commit_path(const PathResult& path) {
    if (!path.success) return;

    for (RREdgeId edge : path.edges) {
        congestion_.increment_edge(edge);
    }
}

} // namespace routing
