#include <iostream>
#include <filesystem>
#include "architecture/parser.h"
#include "netlist/parser.h"
#include "placement/parser.h"
#include "routing/graph_builder.h"

namespace fs = std::filesystem;

int main() {
    std::string data_dir = "../data";
    
    if (!fs::exists(data_dir)) {
        std::cerr << "Directory not found: " << data_dir << std::endl;
        return 1;
    }
    
    std::string arch_file = data_dir + "/k6_frac_N10_mem32K_40nm.xml";
    std::string net_file = data_dir + "/circuito_simples.net";
    std::string place_file = data_dir + "/circuito_simples.place";
    
    if (!fs::exists(arch_file) || !fs::exists(net_file) || !fs::exists(place_file)) {
        std::cerr << "Missing required files" << std::endl;
        return 1;
    }
    
    // Parsear arquivos
    auto fpga_arch = parse_architecture_xml(arch_file);
    auto nets = read_net_file(net_file);
    auto placements = read_place_file(place_file);
    
    // Construir grafo de roteamento
    RoutingGraphBuilder builder;
    RoutingGraph rr_graph = builder.buildGraph(fpga_arch, nets, placements);
    
    // Imprimir estatísticas
    std::cout << "\n====== FINAL STATISTICS ======\n";
    std::cout << "RR Graph Nodes: " << rr_graph.nodes.size() << "\n";
    std::cout << "RR Graph Edges: " << rr_graph.edges.size() << "\n";
    std::cout << "Nets: " << nets.size() << "\n";
    std::cout << "Placements: " << placements.size() << "\n";
    
    // Imprimir primeiros 10 nós
    std::cout << "\nFirst 10 RR Nodes:\n";
    for (int i = 0; i < std::min(10, (int)rr_graph.nodes.size()); ++i) {
        const auto& node = rr_graph.nodes[i];
        std::cout << "  Node " << node.id << ": " << node.name 
                  << " (Type: " << (int)node.type 
                  << ", Pos: " << node.x << "," << node.y 
                  << ", Delay: " << node.delay << ")\n";
    }
    
    return 0;
}