#include "../../include/validation/validate.h"
#include <iostream>
#include <algorithm>

ValidationStats Validator::validateRouting(
    const RoutingGraph& graph,
    const std::vector<RouteTree>& routes,
    const std::vector<Net>& physical_nets
) {
    ValidationStats stats = {true, 0, 0, 0};
    std::map<int, int> node_usage_check;

    for (const auto& route : routes) {
        if (!route.routed || route.nodes.empty()) {
            stats.unrouted_nets++;
            stats.is_valid = false;
            std::cout << "[FALHA] Net " << route.net_id << " nao roteada.\n";
            continue;
        }

        // puxa os source e sink
        const Net& target_net = physical_nets[route.net_id];
        
        // Verifica origem
        if (route.nodes.front() != target_net.driver) {
            stats.is_valid = false;
            std::cout << "[FALHA] Net " << route.net_id << " começa no nó errado. "
                      << "Esperado: " << target_net.driver 
                      << ", Atual: " << route.nodes.front() << "\n";
        }

        // Verifica destino (assume-se que o ultimo no da rota deve ser um dos sinks)
        int last_node = route.nodes.back();
        bool reached_sink = false;
        for (int sink_id : target_net.sinks) {
            if (last_node == sink_id) {
                reached_sink = true;
                break;
            }
        }

        if (!reached_sink) {
            stats.is_valid = false;
            std::cout << "[FALHA] Net " << route.net_id << " nao atingiu nenhum sink valido.\n";
        }

        // verificação de conectividade e uso
        for (size_t i = 0; i < route.nodes.size() - 1; ++i) {
            int current_node = route.nodes[i];
            int next_node = route.nodes[i+1];
            
            node_usage_check[current_node]++;

            const auto& neighbors = graph.getNeighbors(current_node);
            bool edge_exists = false;
            for (int neighbor : neighbors) {
                if (neighbor == next_node) {
                    edge_exists = true;
                    break;
                }
            }

            if (!edge_exists) {
                stats.disconnected_paths++;
                stats.is_valid = false;
                std::cout << "[FALHA] Descontinuidade na Net " << route.net_id << "\n";
            }
        }
        node_usage_check[route.nodes.back()]++;
    }

    // verificação de capacidade
    for (const auto& node : graph.nodes) {
        if (node_usage_check[node.id] > node.capacity) {
            stats.overused_nodes++;
            stats.is_valid = false;
            std::cout << "[FALHA] Congestionamento no No " << node.id 
                      << ": Uso " << node_usage_check[node.id] 
                      << " > Cap " << node.capacity << "\n";
        }
    }

    return stats;
}