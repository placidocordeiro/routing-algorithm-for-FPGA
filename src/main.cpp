#include <iostream>
#include <filesystem>
#include "architecture/parser.h"
#include "netlist/parser.h"
#include "placement/parser.h"
#include "routing/graph_builder.h"
#include "routing/router.h"

namespace fs = std::filesystem;

int main() {
    std::string data_dir = "../data";
    
    auto fpga_arch = parse_architecture_xml(data_dir + "/k6_frac_N10_mem32K_40nm.xml");
    auto nets = read_net_file(data_dir + "/circuito_simples.net");
    auto placements = read_place_file(data_dir + "/circuito_simples.place");
    
    // 1. Construir grafo
    RoutingGraphBuilder builder;
    RoutingGraph rr_graph = builder.buildGraph(fpga_arch, nets, placements);
    
    // 2. Mapear nets para nós físicos
    std::vector<Net> physical_nets;
    builder.mapNetsToPhysicalNodes(nets, placements, fpga_arch, physical_nets, rr_graph);
    
    // 3. Executar routing
    Router router;
    auto routes = router.route(rr_graph, physical_nets);
    
    // 4. Estatísticas
    std::cout << "\n====== RESULTADOS DO ROUTING ======\n";
    int routed_nets = 0;
    float total_delay = 0.0f;
    
    for (const auto& route : routes) {
        if (route.routed) {
            routed_nets++;
            total_delay += route.total_delay;
            std::cout << "Net " << route.net_id 
                      << ": " << route.nodes.size() << " nós"
                      << ", delay: " << route.total_delay << " ns\n";
        }
    }
    
    std::cout << "\nEstatísticas:\n";
    std::cout << "Nets totais: " << nets.size() << "\n";
    std::cout << "Nets roteadas: " << routed_nets << "\n";
    std::cout << "Delay total: " << total_delay << " ns\n";
    std::cout << "Delay médio por net: " 
              << (routed_nets > 0 ? total_delay / routed_nets : 0) << " ns\n";
    
    return 0;
}