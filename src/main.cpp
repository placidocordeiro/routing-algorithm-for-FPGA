<<<<<<< HEAD
#include <iostream>
#include <iomanip>
=======
>>>>>>> 54a53f8767cd7f6f610083ea090775ca1e8daef0
#include <filesystem>
#include <iostream>

#include "architecture/parser.h"
<<<<<<< HEAD

namespace fs = std::filesystem;

void print_separator(const std::string& title = "") {
    std::cout << "\n" << std::string(80, '=') << "\n";
    if (!title.empty()) {
        std::cout << " " << title << "\n";
        std::cout << std::string(80, '=') << "\n";
    }
}

void print_device_info(const Device& device) {
    print_separator("DEVICE INFORMATION");
    std::cout << std::left << std::setw(30) << "R_minW_nmos:" << device.R_minW_nmos << "\n";
    std::cout << std::left << std::setw(30) << "R_minW_pmos:" << device.R_minW_pmos << "\n";
    std::cout << std::left << std::setw(30) << "grid_logic_tile_area:" << device.grid_logic_tile_area << "\n";
    std::cout << std::left << std::setw(30) << "switch_block_type:" << device.switch_block_type << "\n";
    std::cout << std::left << std::setw(30) << "fs:" << device.fs << "\n";
    std::cout << std::left << std::setw(30) << "connection_block_switch:" << device.connection_block_switch << "\n";
}

void print_switches(const std::vector<Switch>& switches) {
    print_separator("SWITCHES (" + std::to_string(switches.size()) + ")");
    
    for (size_t i = 0; i < switches.size(); i++) {
        std::cout << "\n--- Switch " << i+1 << " ---\n";
        std::cout << std::left << std::setw(20) << "  type:" << switches[i].type << "\n";
        std::cout << std::left << std::setw(20) << "  name:" << switches[i].name << "\n";
        std::cout << std::left << std::setw(20) << "  R:" << switches[i].R << "\n";
        std::cout << std::left << std::setw(20) << "  Cin:" << switches[i].Cin << "\n";
        std::cout << std::left << std::setw(20) << "  Cout:" << switches[i].Cout << "\n";
        std::cout << std::left << std::setw(20) << "  Tdel:" << switches[i].Tdel << "\n";
        std::cout << std::left << std::setw(20) << "  mux_trans_size:" << switches[i].mux_trans_size << "\n";
        std::cout << std::left << std::setw(20) << "  buf_size:" << switches[i].buf_size << "\n";
    }
}

void print_segments(const std::vector<Segment>& segments) {
    print_separator("SEGMENTS (" + std::to_string(segments.size()) + ")");
    
    for (size_t i = 0; i < segments.size(); i++) {
        std::cout << "\n--- Segment " << i+1 << " ---\n";
        std::cout << std::left << std::setw(15) << "  freq:" << segments[i].freq << "\n";
        std::cout << std::left << std::setw(15) << "  length:" << segments[i].length << "\n";
        std::cout << std::left << std::setw(15) << "  type:" << segments[i].type << "\n";
        std::cout << std::left << std::setw(15) << "  Rmetal:" << segments[i].Rmetal << "\n";
        std::cout << std::left << std::setw(15) << "  Cmetal:" << segments[i].Cmetal << "\n";
        std::cout << std::left << std::setw(15) << "  mux_name:" << segments[i].mux_name << "\n";
    }
}

void print_directs(const std::vector<Direct>& directs) {
    print_separator("DIRECT CONNECTIONS (" + std::to_string(directs.size()) + ")");
    
    for (size_t i = 0; i < directs.size(); i++) {
        std::cout << "\n--- Direct " << i+1 << " ---\n";
        std::cout << std::left << std::setw(15) << "  name:" << directs[i].name << "\n";
        std::cout << std::left << std::setw(15) << "  input:" << directs[i].input << "\n";
        std::cout << std::left << std::setw(15) << "  output:" << directs[i].output << "\n";
        std::cout << std::left << std::setw(15) << "  x_offset:" << directs[i].x_offset << "\n";
        std::cout << std::left << std::setw(15) << "  y_offset:" << directs[i].y_offset << "\n";
        std::cout << std::left << std::setw(15) << "  z_offset:" << directs[i].z_offset << "\n";
    }
}

void print_tiles(const std::vector<Tile>& tiles) {
    print_separator("TILES (" + std::to_string(tiles.size()) + ")");
    
    for (size_t i = 0; i < tiles.size(); i++) {
        std::cout << "\n--- Tile " << i+1 << ": " << tiles[i].name << " ---\n";
        std::cout << std::left << std::setw(15) << "  name:" << tiles[i].name << "\n";
        std::cout << std::left << std::setw(15) << "  type:" << tiles[i].type << "\n";
        std::cout << std::left << std::setw(15) << "  height:" << tiles[i].height << "\n";
        std::cout << std::left << std::setw(15) << "  area:" << tiles[i].area << "\n";
        std::cout << std::left << std::setw(15) << "  fc_in:" << tiles[i].fc_in << "\n";
        std::cout << std::left << std::setw(15) << "  fc_out:" << tiles[i].fc_out << "\n";
        
        if (!tiles[i].ports.empty()) {
            std::cout << "  ports (" << tiles[i].ports.size() << "):\n";
            for (const auto& port : tiles[i].ports) {
                std::cout << "    - " << std::left << std::setw(10) << port.type 
                         << ": " << std::setw(15) << port.name 
                         << " (pins: " << port.num_pins << ")"
                         << (port.is_clock ? " [CLOCK]" : "") << "\n";
            }
        } else {
            std::cout << "  ports: none\n";
        }
    }
}

void print_summary(const FPGAArchitecture& arch) {
    print_separator("SUMMARY");
    std::cout << "Total switches: " << arch.switches.size() << "\n";
    std::cout << "Total segments: " << arch.segments.size() << "\n";
    std::cout << "Total directs: " << arch.directs.size() << "\n";
    std::cout << "Total tiles: " << arch.tiles.size() << "\n";
    
    // Contagem total de ports
    int total_ports = 0;
    for (const auto& tile : arch.tiles) {
        total_ports += tile.ports.size();
    }
    std::cout << "Total ports across all tiles: " << total_ports << "\n";
}

int main() {
    std::string data_dir = "../data";
    std::string filename = data_dir + "/k6_frac_N10_mem32K_40nm.xml";
    
    std::cout << "\nCarregando arquitetura: " << filename << "\n";
    
    auto fpga_arch = parse_architecture_xml(filename);
    
    // Verificar se a arquitetura foi carregada corretamente
    if (fpga_arch.switches.empty() && fpga_arch.segments.empty() && 
        fpga_arch.tiles.empty()) {
        std::cerr << "ERRO: Falha ao carregar arquitetura ou arquivo vazio!\n";
        return 1;
    }
    
    std::cout << "Arquitetura carregada com sucesso!\n";
    
    // Imprimir todas as informações
    // print_device_info(fpga_arch.device);
    // print_switches(fpga_arch.switches);
    // print_segments(fpga_arch.segments);
    print_directs(fpga_arch.directs);
    // print_tiles(fpga_arch.tiles);
    print_summary(fpga_arch);
    
    std::cout << "\nFim da leitura da arquitetura.\n\n";
    
=======
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

>>>>>>> 54a53f8767cd7f6f610083ea090775ca1e8daef0
    return 0;
}
