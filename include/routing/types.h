#ifndef ROUTING_TYPES_H
#define ROUTING_TYPES_H

#include <string>
#include <vector>
#include <map>
#include <set>

// Tipos de nós do RRGraph
enum class RRNodeType {
    SOURCE,         // Fonte dentro de um bloco
    SINK,           // Sumidouro dentro de um bloco
    OPIN,           // Pino de saída
    IPIN,           // Pino de entrada
    CHANX,          // Segmento horizontal
    CHANY,          // Segmento vertical
    CHAN_WIRE,      // Segmento de fio genérico
    VERTEX,         // Vértice genérico
    EDGE            // Aresta (switches)
};

struct RRNode {
    int id;
    RRNodeType type;
    int x, y;
    int x_low, y_low, x_high, y_high;
    int ptc;
    int capacity;
    int used;  // Quantas nets usando este nó
    float base_cost;
    float delay;
    std::string name;
};

struct RREdge {
    int from_node;
    int to_node;
    int switch_id;
    float delay;
};

struct TimingConstraints {
    float clock_period;
    std::map<int, float> arrival_times;
    std::map<int, float> required_times;
    std::map<int, float> slacks;
};

struct RoutingGraph {
    std::vector<RRNode> nodes;
    std::vector<RREdge> edges;
    std::map<int, std::vector<int>> adjacency_list;     // node_id -> [neighbor_ids]
    std::map<int, std::vector<int>> reverse_adjacency;  // Para busca bidirecional
    TimingConstraints timing;
    
    // Métodos utilitários
    void addNode(const RRNode& node) {
        nodes.push_back(node);
    }
    
    void addEdge(const RREdge& edge) {
        edges.push_back(edge);
        adjacency_list[edge.from_node].push_back(edge.to_node);
        reverse_adjacency[edge.to_node].push_back(edge.from_node);
    }
    
    // Novo: obter nós adjacentes
    const std::vector<int>& getNeighbors(int node_id) const {
        static const std::vector<int> empty;
        auto it = adjacency_list.find(node_id);
        return it != adjacency_list.end() ? it->second : empty;
    }
    
    // Novo: resetar uso
    void resetUsage() {
        for (auto& node : nodes) {
            node.used = 0;
        }
    }
};

struct RouteTree {
    int net_id;
    std::vector<int> nodes;  // IDs dos nós usados na rota
    float total_delay;
    bool routed;
};

#endif