#ifndef ROUTING_TYPES_H
#define ROUTING_TYPES_H

#include <string>
#include <vector>
#include <map>

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
    int x_low, y_low, x_high, y_high;  // Para segmentos
    int ptc;  // Pin ou track class
    int capacity;
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
    std::map<int, float> arrival_times;  // node_id -> arrival_time
    std::map<int, float> required_times; // node_id -> required_time
    std::map<int, float> slacks;         // node_id -> slack
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
};

#endif