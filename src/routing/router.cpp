#include "routing/router.h"
#include <algorithm>
#include <cassert>
#include <iostream>
#include <limits>
#include <queue>
#include <unordered_map>

struct DijkstraNode {
    int id;
    float cost;

    bool operator>(const DijkstraNode& other) const
    {
        return cost > other.cost;
    }
};

std::vector<RouteTree> Router::route(
    const RoutingGraph& graph,
    const std::vector<Net>& nets,
    const std::vector<int>& net_order)
{
    std::vector<RouteTree> routes;
    routes.reserve(nets.size());

    for (int idx : net_order) {
        const Net& net = nets[idx];

        RouteTree tree;
        tree.net_id = net.id;
        tree.total_delay = 0.0f;
        tree.routed = false;

        if (net.driver >= 0 && !net.sinks.empty()) {

            auto path = findPath(graph, net.driver, net.sinks);

            if (!path.empty()) {
                tree.nodes = path;
                tree.routed = true;

                float criticality = 0.5f;

                for (int node_id : path) {
                    if (node_id < graph.nodes.size()) {
                        const RRNode& node = graph.nodes[node_id];
                        tree.total_delay += getNodeCost(node, criticality);
                    }
                }
            }
        }

        routes.push_back(tree);
    }

    return routes;
}

std::vector<int> Router::findPath(
    const RoutingGraph& graph,
    int source_id,
    const std::vector<int>& sinks_ids)
{
    // Dijkstra simplificado para múltiplos sinks
    std::priority_queue<DijkstraNode, std::vector<DijkstraNode>,
        std::greater<DijkstraNode>>
        pq;

    std::unordered_map<int, float> dist;
    std::unordered_map<int, int> prev;
    std::set<int> sinks_set(sinks_ids.begin(), sinks_ids.end());
    std::vector<int> path;

    // Inicializar distâncias
    for (size_t i = 0; i < graph.nodes.size(); ++i) {
        dist[i] = std::numeric_limits<float>::infinity();
    }

    dist[source_id] = 0.0f;
    pq.push({ source_id, 0.0f });

    int target_reached = -1;

    // Executar Dijkstra
    while (!pq.empty()) {
        auto current = pq.top();
        pq.pop();

        // Se chegamos em algum sink, parar
        if (sinks_set.find(current.id) != sinks_set.end()) {
            target_reached = current.id;
            break;
        }

        // Explorar vizinhos
        const auto& neighbors = graph.getNeighbors(current.id);
        for (int neighbor_id : neighbors) {
            if (neighbor_id < graph.nodes.size()) {
                const auto& neighbor = graph.nodes[neighbor_id];

                // Custo: atraso do nó + atraso da aresta
                float edge_delay = 0.0f;
                for (const auto& edge : graph.edges) {
                    if (edge.from_node == current.id && edge.to_node == neighbor_id) {
                        edge_delay = edge.delay;
                        break;
                    }
                }

                float new_cost = current.cost + neighbor.delay + edge_delay;

                if (new_cost < dist[neighbor_id]) {
                    dist[neighbor_id] = new_cost;
                    prev[neighbor_id] = current.id;
                    pq.push({ neighbor_id, new_cost });
                }
            }
        }
    }

    // Reconstruir caminho
    if (target_reached != -1) {
        int current = target_reached;
        while (current != source_id) {
            path.push_back(current);
            current = prev[current];
        }
        path.push_back(source_id);
        std::reverse(path.begin(), path.end());
    }

    return path;
}

float Router::getNodeCost(const RRNode& node, float criticality)
{
    assert(criticality >= 0.0f && criticality <= 1.0f);
    assert(node.capacity > 0);
    // Custo base + penalidade por congestionamento
    float base_cost = node.base_cost > 0 ? node.base_cost : 1.0f;
    float congestion_cost = 1.0f + node.used;

    assert(congestion_cost >= 0.0f);
    // Balanceamento timing/congestionamento
    return (criticality * node.delay) + ((1.0f - criticality) * base_cost * congestion_cost);
}