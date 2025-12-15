#include "../../include/routing/graph_builder.h"
#include <iostream>
#include <sstream>
#include <cmath>

RoutingGraph RoutingGraphBuilder::buildGraph(
    const FPGAArchitecture& arch,
    const std::vector<Net>& nets,
    const std::vector<Placement>& placements
) {
    RoutingGraph graph;
    
    // 1. Criar nós fictícios para teste
    createTestNodes(graph, nets);
    
    std::cout << "RRGraph built with " << graph.nodes.size() 
              << " nodes and " << graph.edges.size() 
              << " edges" << std::endl;
    
    return graph;
}

void RoutingGraphBuilder::createTestNodes(RoutingGraph& graph, const std::vector<Net>& nets) {
    // Criar nós de teste conectados (grid 3x3 fictício)
    int node_id = 0;
    
    // Criar 20 nós fictícios para teste
    for (int i = 0; i < 20; ++i) {
        RRNode node;
        node.id = node_id++;
        node.type = (i % 2 == 0) ? RRNodeType::IPIN : RRNodeType::CHANX;
        node.x = i % 5;
        node.y = i / 5;
        node.capacity = 1;
        node.used = 0;
        node.base_cost = 1.0f;
        node.delay = 0.1f + (i * 0.02f);  // Delays variados
        node.name = "TEST_NODE_" + std::to_string(i);
        
        graph.addNode(node);
    }
    
    // Criar arestas de conexão
    for (int i = 0; i < 19; ++i) {
        RREdge edge;
        edge.from_node = i;
        edge.to_node = i + 1;
        edge.switch_id = 0;
        edge.delay = 0.05f;
        graph.addEdge(edge);
    }
    
    // Adicionar algumas conexões cruzadas
    graph.addEdge({0, 5, 0, 0.05f});
    graph.addEdge({5, 10, 0, 0.05f});
    graph.addEdge({10, 15, 0, 0.05f});
}

void RoutingGraphBuilder::createTileNodes(
    const Tile& tile,
    int x, 
    int y,
    const FPGAArchitecture& arch,
    RoutingGraph& graph
) {
    // Implementação simplificada - criar nós básicos para cada porta
    for (const auto& port : tile.ports) {
        for (int pin_idx = 0; pin_idx < port.num_pins; ++pin_idx) {
            RRNode node;
            node.id = graph.nodes.size();
            
            // Determinar tipo
            if (port.type == "input") {
                node.type = RRNodeType::IPIN;
            } else if (port.type == "output") {
                node.type = RRNodeType::OPIN;
            } else if (port.type == "clock") {
                node.type = RRNodeType::IPIN;
            } else {
                node.type = RRNodeType::VERTEX;
            }
            
            node.x = x;
            node.y = y;
            node.capacity = 1;
            node.used = 0;
            node.base_cost = 1.0f;
            node.delay = 0.1f;
            node.name = tile.name + "_" + port.name;
            
            if (port.num_pins > 1) {
                node.name += "[" + std::to_string(pin_idx) + "]";
            }
            
            graph.addNode(node);
            
            // Armazenar mapeamento
            std::string pin_key = port.name;
            if (port.num_pins > 1) {
                pin_key += "[" + std::to_string(pin_idx) + "]";
            }
            pin_node_map_[std::make_tuple(x, y, tile.name, pin_key)] = node.id;
        }
    }
    
    // Criar nós SOURCE e SINK básicos
    RRNode source_node;
    source_node.id = graph.nodes.size();
    source_node.type = RRNodeType::SOURCE;
    source_node.x = x;
    source_node.y = y;
    source_node.capacity = 1;
    source_node.used = 0;
    source_node.base_cost = 1.0f;
    source_node.delay = 0.0f;
    source_node.name = tile.name + "_SOURCE";
    graph.addNode(source_node);
    
    RRNode sink_node;
    sink_node.id = graph.nodes.size();
    sink_node.type = RRNodeType::SINK;
    sink_node.x = x;
    sink_node.y = y;
    sink_node.capacity = 1;
    sink_node.used = 0;
    sink_node.base_cost = 1.0f;
    sink_node.delay = 0.0f;
    sink_node.name = tile.name + "_SINK";
    graph.addNode(sink_node);
}

void RoutingGraphBuilder::mapNetsToPhysicalNodes(
    const std::vector<Net>& logical_nets,
    const std::vector<Placement>& placements,
    const FPGAArchitecture& arch,
    std::vector<Net>& physical_nets,
    RoutingGraph& graph
) {
    // Mapeamento simplificado: atribuir nós fictícios
    for (size_t i = 0; i < logical_nets.size(); ++i) {
        Net physical_net = logical_nets[i];
        
        // Driver no primeiro nó, sinks distribuídos
        if (graph.nodes.size() > 0) {
            physical_net.driver = 0;  // Driver no nó 0
            
            // Sinks em outros nós
            physical_net.sinks.clear();
            for (size_t j = 1; j < std::min((size_t)3, graph.nodes.size()); ++j) {
                physical_net.sinks.push_back(j);
            }
        }
        
        physical_nets.push_back(physical_net);
    }
}