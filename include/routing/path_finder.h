#pragma once

#include "rr_graph_view.h"
#include <vector>
#include <queue>
#include <map>
#include <cmath>

namespace routing {

// Estrutura para representar caminho encontrado
struct PathResult {
    bool success;                      // Path foi encontrado?
    std::vector<RREdgeId> edges;      // Arestas do caminho (source -> sink)
    float total_cost;

    PathResult() : success(false), total_cost(0) {}
};

// Parâmetros de busca de caminho
struct PathFindingParams {
    RRNodeId source;
    RRNodeId sink;
    float congestion_weight = 1.0f;
    int max_iterations = 100000;      // Limite de explorações para evitar loops infinitos
};

// Gerenciador de uso de recursos (congestionamento)
class CongestionMap {
public:
    CongestionMap() : capacity_per_edge_(1) {}

    void set_edge_usage(RREdgeId edge_id, int usage) {
        edge_usage_[edge_id] = usage;
    }

    void reset_edge(RREdgeId edge_id) {
        edge_usage_.erase(edge_id);
    }

    int get_edge_usage(RREdgeId edge_id) const {
        auto it = edge_usage_.find(edge_id);
        return (it != edge_usage_.end()) ? it->second : 0;
    }

    void increment_edge(RREdgeId edge_id) {
        edge_usage_[edge_id]++;
    }

    float get_congestion_cost(RREdgeId edge_id) const {
        int usage = get_edge_usage(edge_id);
        if (usage >= capacity_per_edge_) {
            // Penalidade alta para violação
            return 1e6f;
        }
        // Custo suave baseado em utilização
        float util = static_cast<float>(usage) / capacity_per_edge_;
        return util * util;  // Quadrático
    }

    void clear() {
        edge_usage_.clear();
    }

private:
    std::map<RREdgeId, int> edge_usage_;
    int capacity_per_edge_;
};

// Busca de caminho usando Dijkstra com custo de congestionamento
class PathFinder {
public:
    PathFinder(const RRGraphView& rr_graph, CongestionMap& congestion)
        : rr_graph_(rr_graph), congestion_(congestion) {}

    // Encontra caminho de source para sink usando Dijkstra
    PathResult find_path(const PathFindingParams& params);

    // Marca todas as arestas do caminho como usadas
    void commit_path(const PathResult& path);

private:
    const RRGraphView& rr_graph_;
    CongestionMap& congestion_;

    // Estrutura interna para nó explorado no heap
    struct ExploredNode {
        RRNodeId node_id;
        float cost;
        RREdgeId came_from_edge;

        bool operator>(const ExploredNode& other) const {
            return cost > other.cost;  // Min-heap
        }
    };

    // Calcula custo heurístico para A* (Manhattan distance)
    float heuristic_cost(RRNodeId current, RRNodeId target) const;

    // Calcula custo total de usar uma aresta
    float edge_cost(RREdgeId edge_id, float congestion_weight) const;
};

} // namespace routing
