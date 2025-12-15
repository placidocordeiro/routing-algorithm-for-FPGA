#include "routing/router.h"
#include <queue>
#include <limits>
#include <unordered_map>
#include <iostream>
#include <algorithm>

struct DijkstraNode {
    int id;
    float cost;
    
    bool operator>(const DijkstraNode& other) const {
        return cost > other.cost;
    }
};

std::vector<RouteTree> Router::route(
    const RoutingGraph& graph,
    const std::vector<Net>& nets
) {
    std::vector<RouteTree> results;
    
    // Para cada net, rotear individualmente
    for (const auto& net : nets) {
        RouteTree route_tree;
        route_tree.net_id = net.id;
        route_tree.total_delay = 0.0f;
        route_tree.routed = false;
        
        std::cout << "Roteando net " << net.name 
                  << " (driver: " << net.driver 
                  << ", sinks: " << net.sinks.size() << ")" << std::endl;
        
        // Verificar se temos driver e sinks válidos
        if (net.driver >= 0 && !net.sinks.empty()) {
            // Encontrar caminho usando Dijkstra simplificado
            auto path = findPath(graph, net.driver, net.sinks);
            
            if (!path.empty()) {
                route_tree.nodes = path;
                route_tree.routed = true;
                
                // Calcular atraso total (simplificado)
                for (int node_id : path) {
                    if (node_id < graph.nodes.size()) {
                        route_tree.total_delay += graph.nodes[node_id].delay;
                    }
                }
                
                std::cout << "  Net roteada com " << path.size() 
                          << " nós, delay: " << route_tree.total_delay 
                          << " ns" << std::endl;
            } else {
                std::cout << "  ERRO: Net não pôde ser roteada!" << std::endl;
            }
        } else {
            std::cout << "  Net inválida (driver ou sinks faltando)" << std::endl;
        }
        
        results.push_back(route_tree);
    }
    
    return results;
}

std::vector<int> Router::findPath(
    const RoutingGraph& graph,
    int source_id,
    const std::vector<int>& sinks_ids
) {
    // Dijkstra simplificado para múltiplos sinks
    std::priority_queue<DijkstraNode, std::vector<DijkstraNode>, 
                       std::greater<DijkstraNode>> pq;
    
    std::unordered_map<int, float> dist;
    std::unordered_map<int, int> prev;
    std::set<int> sinks_set(sinks_ids.begin(), sinks_ids.end());
    std::vector<int> path;
    
    // Inicializar distâncias
    for (size_t i = 0; i < graph.nodes.size(); ++i) {
        dist[i] = std::numeric_limits<float>::infinity();
    }
    
    dist[source_id] = 0.0f;
    pq.push({source_id, 0.0f});
    
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
                    pq.push({neighbor_id, new_cost});
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

float Router::getNodeCost(const RRNode& node, float criticality) {
    // Custo base + penalidade por congestionamento
    float base_cost = node.base_cost > 0 ? node.base_cost : 1.0f;
    float congestion_cost = 1.0f + node.used;
    
    // Balanceamento timing/congestionamento
    return (criticality * node.delay) + ((1.0f - criticality) * base_cost * congestion_cost);
}