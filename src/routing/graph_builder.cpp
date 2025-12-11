#include "routing/graph_builder.h"
#include <iostream>
#include <sstream>
#include <cmath>

RoutingGraph RoutingGraphBuilder::buildGraph(
    const FPGAArchitecture& arch,
    const std::vector<Net>& nets,
    const std::vector<Placement>& placements
) {
    RoutingGraph graph;
    
    // Determinar tamanho do grid
    int max_x = 0, max_y = 0;
    for (const auto& place : placements) {
        max_x = std::max(max_x, place.x);
        max_y = std::max(max_y, place.y);
    }
    int grid_width = max_x + 1;
    int grid_height = max_y + 1;
    
    std::cout << "Building RRGraph for grid: " << grid_width << " x " << grid_height << std::endl;
    
    // Criar nós para cada tile em cada posição
    for (const auto& place : placements) {
        // Encontrar o tipo de tile correspondente
        for (const auto& tile : arch.tiles) {
            if (tile.name == place.block_name) {
                createTileNodes(tile, place.x, place.y, arch, graph);
                break;
            }
        }
    }
    
    // Criar nós de canal (segmentos de fio)
    createChannelNodes(arch, grid_width, grid_height, graph);
    
    // Criar conexões via switches
    createSwitchConnections(arch, graph);
    
    // Criar conexões diretas (directs)
    createDirectConnections(arch, graph);
    
    // Mapear nets para nós físicos
    std::vector<Net> physical_nets;
    mapNetsToPhysicalNodes(nets, placements, arch, physical_nets, graph);
    
    // Configurar constraints de tempo (simplificado)
    graph.timing.clock_period = 10.0f;  // Exemplo: 10ns
    
    std::cout << "RRGraph built successfully!" << std::endl;
    std::cout << "  - Nodes: " << graph.nodes.size() << std::endl;
    std::cout << "  - Edges: " << graph.edges.size() << std::endl;
    std::cout << "  - Nets: " << physical_nets.size() << std::endl;
    
    return graph;
}

void RoutingGraphBuilder::createTileNodes(
    const Tile& tile,
    int x, 
    int y,
    const FPGAArchitecture& arch,
    RoutingGraph& graph
) {
    // Para cada porta no tile
    for (const auto& port : tile.ports) {
        for (int pin_idx = 0; pin_idx < port.num_pins; ++pin_idx) {
            RRNode node;
            node.id = graph.nodes.size();
            node.x = x;
            node.y = y;
            node.capacity = 1;
            
            // Determinar tipo baseado na porta
            if (port.type == "input") {
                node.type = RRNodeType::IPIN;
                node.name = tile.name + "[" + std::to_string(x) + "," + std::to_string(y) + "]." + 
                           port.name + "[" + std::to_string(pin_idx) + "]";
            } else if (port.type == "output") {
                node.type = RRNodeType::OPIN;
                node.name = tile.name + "[" + std::to_string(x) + "," + std::to_string(y) + "]." + 
                           port.name + "[" + std::to_string(pin_idx) + "]";
            } else if (port.type == "clock") {
                node.type = RRNodeType::IPIN;
                node.name = tile.name + "[" + std::to_string(x) + "," + std::to_string(y) + "]." + 
                           "clk[" + std::to_string(pin_idx) + "]";
            }
            
            // Calcular atraso (simplificado)
            node.delay = 0.1f;  // Atraso fixo para pinos
            
            // Armazenar no mapeamento para referência futura
            std::string pin_key = port.name;
            if (port.num_pins > 1) {
                pin_key += "[" + std::to_string(pin_idx) + "]";
            }
            pin_node_map_[std::make_tuple(x, y, tile.name, pin_key)] = node.id;
            
            graph.addNode(node);
        }
    }
    
    // Criar nós SOURCE e SINK para o tile
    RRNode source_node;
    source_node.id = graph.nodes.size();
    source_node.type = RRNodeType::SOURCE;
    source_node.x = x;
    source_node.y = y;
    source_node.capacity = 1;
    source_node.delay = 0.0f;
    source_node.name = tile.name + "[" + std::to_string(x) + "," + std::to_string(y) + "].SOURCE";
    graph.addNode(source_node);
    
    RRNode sink_node;
    sink_node.id = graph.nodes.size();
    sink_node.type = RRNodeType::SINK;
    sink_node.x = x;
    sink_node.y = y;
    sink_node.capacity = 1;
    sink_node.delay = 0.0f;
    sink_node.name = tile.name + "[" + std::to_string(x) + "," + std::to_string(y) + "].SINK";
    graph.addNode(sink_node);
}

void RoutingGraphBuilder::createChannelNodes(
    const FPGAArchitecture& arch,
    int grid_width,
    int grid_height,
    RoutingGraph& graph
) {
    // Para cada segmento definido na arquitetura
    for (const auto& segment : arch.segments) {
        // Criar segmentos horizontais (CHANX)
        for (int y = 0; y < grid_height; ++y) {
            for (int x = 0; x < grid_width - 1; ++x) {
                for (int track = 0; track < 1; ++track) {  // Simplificado: 1 track por segmento
                    RRNode node;
                    node.id = graph.nodes.size();
                    node.type = RRNodeType::CHANX;
                    node.x_low = x;
                    node.y_low = y;
                    node.x_high = x + segment.length - 1;
                    node.y_high = y;
                    node.ptc = track;
                    node.capacity = 1;
                    node.delay = segment.Rmetal * segment.Cmetal * segment.length;  // Modelo simplificado
                    node.name = "CHANX[" + std::to_string(x) + "-" + 
                               std::to_string(node.x_high) + "," + 
                               std::to_string(y) + "].L" + 
                               std::to_string(segment.length);
                    
                    graph.addNode(node);
                }
            }
        }
        
        // Criar segmentos verticais (CHANY)
        for (int x = 0; x < grid_width; ++x) {
            for (int y = 0; y < grid_height - 1; ++y) {
                for (int track = 0; track < 1; ++track) {
                    RRNode node;
                    node.id = graph.nodes.size();
                    node.type = RRNodeType::CHANY;
                    node.x_low = x;
                    node.y_low = y;
                    node.x_high = x;
                    node.y_high = y + segment.length - 1;
                    node.ptc = track;
                    node.capacity = 1;
                    node.delay = segment.Rmetal * segment.Cmetal * segment.length;
                    node.name = "CHANY[" + std::to_string(x) + "," + 
                               std::to_string(y) + "-" + 
                               std::to_string(node.y_high) + "].L" + 
                               std::to_string(segment.length);
                    
                    graph.addNode(node);
                }
            }
        }
    }
}

void RoutingGraphBuilder::createSwitchConnections(
    const FPGAArchitecture& arch,
    RoutingGraph& graph
) {
    // Conectar OPINs a segmentos de canal (simplificado)
    for (const auto& node : graph.nodes) {
        if (node.type == RRNodeType::OPIN) {
            // Conectar a segmentos horizontais na mesma posição
            for (const auto& target_node : graph.nodes) {
                if (target_node.type == RRNodeType::CHANX && 
                    target_node.x_low == node.x && 
                    target_node.y_low == node.y) {
                    
                    RREdge edge;
                    edge.from_node = node.id;
                    edge.to_node = target_node.id;
                    edge.switch_id = 0;  // Primeiro switch
                    edge.delay = 0.05f;  // Atraso do switch
                    
                    graph.addEdge(edge);
                }
            }
        }
        
        // Conectar segmentos entre si (switch block)
        if (node.type == RRNodeType::CHANX) {
            for (const auto& target_node : graph.nodes) {
                if (target_node.type == RRNodeType::CHANY && 
                    target_node.x_low == node.x_high && 
                    target_node.y_low == node.y_low) {
                    
                    // Conexão em L
                    RREdge edge;
                    edge.from_node = node.id;
                    edge.to_node = target_node.id;
                    edge.switch_id = 0;
                    edge.delay = 0.05f;
                    
                    graph.addEdge(edge);
                }
            }
        }
    }
}

void RoutingGraphBuilder::createDirectConnections(
    const FPGAArchitecture& arch,
    RoutingGraph& graph
) {
    // Implementar conexões diretas (simplificado)
    for (const auto& direct : arch.directs) {
        //  Lógica a ser implementada para conectar pins diretamente
        // baseado nos offsets e tipos de pinos
    }
}

void RoutingGraphBuilder::mapNetsToPhysicalNodes(
    const std::vector<Net>& logical_nets,
    const std::vector<Placement>& placements,
    const FPGAArchitecture& arch,
    std::vector<Net>& physical_nets,
    RoutingGraph& graph
) {
    // Mapeamento simplificado: assumindo que cada net tem um driver e sinks
    // Futuramente, precisaremos do mapeamento exato de pinos
    
    for (const auto& logical_net : logical_nets) {
        Net physical_net = logical_net;
        
        // Resetar mapeamentos físicos
        physical_net.driver = -1;
        physical_net.sinks.clear();
        
        // Aqui deve ser implementada a lógica para mapear os nomes lógicos
        // para IDs de nós físicos baseado no placement
        
        physical_nets.push_back(physical_net);
    }
}