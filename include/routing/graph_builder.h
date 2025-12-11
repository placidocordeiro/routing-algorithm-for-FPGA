#ifndef ROUTING_GRAPH_BUILDER_H
#define ROUTING_GRAPH_BUILDER_H

#include "architecture/types.h"
#include "netlist/types.h"
#include "placement/types.h"
#include "routing/types.h"
#include <vector>
#include <map>

class RoutingGraphBuilder {
public:
    RoutingGraphBuilder() = default;
    
    // Constrói o grafo de roteamento completo
    RoutingGraph buildGraph(
        const FPGAArchitecture& arch,
        const std::vector<Net>& nets,
        const std::vector<Placement>& placements
    );
    
private:
    // Métodos auxiliares
    void createTileNodes(
        const Tile& tile,
        int x, 
        int y,
        const FPGAArchitecture& arch,
        RoutingGraph& graph
    );
    
    void createChannelNodes(
        const FPGAArchitecture& arch,
        int grid_width,
        int grid_height,
        RoutingGraph& graph
    );
    
    void createSwitchConnections(
        const FPGAArchitecture& arch,
        RoutingGraph& graph
    );
    
    void createDirectConnections(
        const FPGAArchitecture& arch,
        RoutingGraph& graph
    );
    
    void mapNetsToPhysicalNodes(
        const std::vector<Net>& logical_nets,
        const std::vector<Placement>& placements,
        const FPGAArchitecture& arch,
        std::vector<Net>& physical_nets,
        RoutingGraph& graph
    );
    
    // Mapeamentos auxiliares
    std::map<std::tuple<int, int, std::string, std::string>, int> pin_node_map_;  
    // (x, y, tile_type, pin_name) -> node_id

    std::map<std::string, int> tile_type_count_;
};

#endif 