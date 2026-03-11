#include <iostream>
#include <iomanip>
#include <filesystem>
#include "architecture/parser.h"

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
    
    return 0;
}