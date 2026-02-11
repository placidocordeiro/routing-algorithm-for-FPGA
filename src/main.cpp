#include <filesystem>
#include <iostream>

#include "architecture/parser.h"
#include "netlist/parser.h"
#include "placement/parser.h"

#include "routing/brkga.h"
#include "routing/graph_builder.h"
#include "routing/router.h"

#include "validation/validate.h"

namespace fs = std::filesystem;

int main()
{
    std::string data_dir = "../data";

    // ===============================
    //      Parsing dos arquivos
    // ===============================
    auto fpga_arch = parse_architecture_xml(
        data_dir + "/k6_frac_N10_mem32K_40nm.xml");

    auto nets = read_net_file(
        data_dir + "/circuito_simples.net");

    auto placements = read_place_file(
        data_dir + "/circuito_simples.place");

    // ===============================
    //     Construção do grafo RR
    // ===============================
    RoutingGraphBuilder builder;
    RoutingGraph rr_graph = builder.buildGraph(fpga_arch, nets, placements);

    // ===============================
    //    Mapeamento lógico → físico
    // ===============================
    std::vector<Net> physical_nets;
    builder.mapNetsToPhysicalNodes(
        nets, placements, fpga_arch, physical_nets, rr_graph);

    // ===============================
    //  BRKGA para ordenação das nets
    // ===============================
    BRKGAParams params;
    params.population_size = 40;
    params.elite_size = 8;
    params.max_generations = 60;
    params.inheritance_prob = 0.7f;

    BRKGA brkga(params);

    std::vector<int> best_order;
    bool success = brkga.run(rr_graph, physical_nets, best_order);

    if (success) {
        std::cout << "\nBRKGA encontrou uma ordenação válida!\n";
    } else {
        std::cout << "\nBRKGA terminou sem solução perfeita, usando melhor tentativa.\n";
    }

    // ===============================
    //        Roteamento final
    // ===============================
    Router router;
    auto routes = router.route(rr_graph, physical_nets, best_order);

    // ===============================
    //            Validação
    // ===============================
    std::cout << "\n====== VALIDACAO ======\n";

    auto val_stats = Validator::validateRouting(
        rr_graph, routes, physical_nets);

    if (val_stats.is_valid) {
        std::cout << "SUCESSO: Roteamento valido e legal!\n";
    } else {
        std::cout << "ERRO: O roteamento possui conflitos ou falhas.\n";
        std::cout << "- Nós com congestionamento: "
                  << val_stats.overused_nodes << "\n";
        std::cout << "- Caminhos descontínuos: "
                  << val_stats.disconnected_paths << "\n";
        std::cout << "- Nets não roteadas: "
                  << val_stats.unrouted_nets << "\n";
    }

    // ===============================
    //       Estatísticas finais
    // ===============================
    std::cout << "\n====== RESULTADOS DO ROUTING ======\n";

    int routed_nets = 0;
    float total_delay = 0.0f;

    for (const auto& route : routes) {
        if (route.routed) {
            routed_nets++;
            total_delay += route.total_delay;

            std::cout << "Net " << route.net_id
                      << ": " << route.nodes.size() << " nós"
                      << ", custo: " << route.total_delay << "\n";
        }
    }

    std::cout << "\nEstatísticas:\n";
    std::cout << "Nets totais: " << physical_nets.size() << "\n";
    std::cout << "Nets roteadas: " << routed_nets << "\n";
    std::cout << "Custo total: " << total_delay << "\n";
    std::cout << "Custo médio por net: "
              << (routed_nets > 0 ? total_delay / routed_nets : 0)
              << "\n";

    return 0;
}
