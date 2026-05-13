#pragma once

#include "rr_graph_view.h"
#include <vector>
#include <queue>
#include <map>

namespace routing {

struct PathResult {
    bool success;
    std::vector<RREdgeId> edges;   // [source -> sink]
    std::vector<RRNodeId> nodes;   // [source, ..., sink]
    float total_cost;

    PathResult() : success(false), total_cost(0) {}
};

struct PathFindingParams {
    RRNodeId source;
    RRNodeId sink;
    int max_iterations = 100000;
};

class PathFinder {
public:
    PathFinder(const RRGraphView& rr_graph) : rr_graph_(rr_graph) {}

    // Dijkstra com hard-capacity: nós com occ >= capacity são descartados,
    // exceto o próprio sink alvo.
    PathResult find_path(const PathFindingParams& params);

private:
    const RRGraphView& rr_graph_;

    struct ExploredNode {
        RRNodeId node_id;
        float cost;
        RREdgeId came_from_edge;

        bool operator>(const ExploredNode& other) const {
            return cost > other.cost;
        }
    };
};

} // namespace routing
