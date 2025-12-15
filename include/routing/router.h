#ifndef ROUTING_ROUTER_H
#define ROUTING_ROUTER_H

#include "./types.h"
#include "../netlist/types.h"

class Router {
public:
    Router() = default;
    
    // Algoritmo de routing básico
    std::vector<RouteTree> route(
        const RoutingGraph& graph,
        const std::vector<Net>& nets
    );
    
private:
    // Dijkstra simplificado
    std::vector<int> findPath(
        const RoutingGraph& graph,
        int source_id,
        const std::vector<int>& sinks_ids
    );
    
    // Calcular custo considerando congestionamento
    float getNodeCost(const RRNode& node, float criticality);
};

#endif