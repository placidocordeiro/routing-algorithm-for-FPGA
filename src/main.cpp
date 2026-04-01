#include <iostream>
#include <iomanip>
#include <chrono>
#include "globals.h"
#include "vpr_context/loader.h"
#include "routing/steiner_router.h"
#include "route_common.h"
#include "route_utils.h"
#include "router_stats.h"
#include "check_route.h"
#include "vpr_api.h"

// Retorna wirelength e feasibility do contexto de roteamento atual
static void collect_metrics(const ClusteredNetlist& nlist,
                            bool& feasible,
                            size_t& wl_used,
                            size_t& wl_available,
                            float& wl_ratio) {
    feasible     = feasible_routing();
    wl_available = calculate_wirelength_available();
    WirelengthInfo wl = calculate_wirelength_info((const Netlist<>&)nlist, wl_available);
    wl_used  = wl.used_wirelength();
    wl_ratio = wl.used_wirelength_ratio();
}

int main(int argc, const char* argv[]) {
    if (argc < 5) {
        std::cerr << "Uso: " << argv[0]
                  << " <arch.xml> <circuito.blif> <circuito.net> <circuito.place>\n";
        return 1;
    }

    t_options   options;
    t_vpr_setup vpr_setup;
    t_arch      arch;

    load_vpr_context(argv[1], argv[2], argv[3], argv[4],
                     options, vpr_setup, arch);

    auto& cluster_ctx = g_vpr_ctx.clustering();
    auto& device_ctx  = g_vpr_ctx.device();

    const auto& nlist    = cluster_ctx.clb_nlist;
    const auto& rr_graph = device_ctx.rr_graph;

    std::cout << "Blocos : " << nlist.blocks().size() << "\n";
    std::cout << "Nets   : " << nlist.nets().size()   << "\n";
    std::cout << "Nós RR : " << rr_graph.nodes().size()  << "\n\n";

    // =========================================================================
    // PASSO 1 — Nosso algoritmo (Steiner Tree + Dijkstra)
    // =========================================================================
    std::cout << ">>> Executando Steiner Router...\n";
    routing::SteinerRouter router(1.0f);

    auto t0 = std::chrono::steady_clock::now();
    auto routing_result = router.route(nlist, rr_graph);
    auto t1 = std::chrono::steady_clock::now();

    double our_time = std::chrono::duration<double>(t1 - t0).count();
    routing_result.print_summary();

    // Validar estrutura das rotas segundo o VTR
    // Se falhar com VTR_ASSERT, há problema estrutural nos RouteTrees
    std::cout << "\nValidando rotas (check_route)...\n";
    try {
        check_route((const Netlist<>&)nlist, DETAILED, e_check_route_option::FULL, /*is_flat=*/false);
        std::cout << "check_route: OK\n";
    } catch (...) {
        std::cout << "check_route: FALHOU (rotas com problema estrutural)\n";
    }

    bool our_feasible;
    size_t our_wl_used, our_wl_avail;
    float our_wl_ratio;
    collect_metrics(nlist, our_feasible, our_wl_used, our_wl_avail, our_wl_ratio);

    // =========================================================================
    // PASSO 2 — VTR nativo (Pathfinder iterativo)
    // vpr_route_flow reinicializa internamente: não precisa reset manual
    // =========================================================================
    std::cout << "\n>>> Executando VTR Router nativo...\n";

    // Garantir que a etapa de roteamento será executada
    vpr_setup.RouterOpts.doRouting = STAGE_DO;

    auto t2 = std::chrono::steady_clock::now();
    RouteStatus vpr_status = vpr_route_flow((const Netlist<>&)nlist, vpr_setup, arch, /*is_flat=*/false);
    auto t3 = std::chrono::steady_clock::now();

    double vpr_time = std::chrono::duration<double>(t3 - t2).count();

    bool vpr_feasible;
    size_t vpr_wl_used, vpr_wl_avail;
    float vpr_wl_ratio;
    collect_metrics(nlist, vpr_feasible, vpr_wl_used, vpr_wl_avail, vpr_wl_ratio);

    // =========================================================================
    // PASSO 3 — Tabela comparativa
    // =========================================================================
    const int W = 22;
    std::cout << "\n";
    std::cout << std::string(60, '=') << "\n";
    std::cout << "  COMPARAÇÃO DE ROTEAMENTO\n";
    std::cout << std::string(60, '=') << "\n";
    std::cout << std::left
              << std::setw(W) << ""
              << std::setw(W) << "Steiner"
              << std::setw(W) << "VTR Nativo"
              << "\n";
    std::cout << std::string(60, '-') << "\n";

    std::cout << std::setw(W) << "Tempo (s)"
              << std::setw(W) << std::fixed << std::setprecision(3) << our_time
              << std::setw(W) << vpr_time << "\n";

    std::cout << std::setw(W) << "WL usado"
              << std::setw(W) << our_wl_used
              << std::setw(W) << vpr_wl_used << "\n";

    std::cout << std::setw(W) << "WL disponivel"
              << std::setw(W) << our_wl_avail
              << std::setw(W) << vpr_wl_avail << "\n";

    std::cout << std::setw(W) << "Utilizacao (%)"
              << std::setw(W) << std::fixed << std::setprecision(1) << (our_wl_ratio * 100.0f)
              << std::setw(W) << (vpr_wl_ratio * 100.0f) << "\n";

    std::cout << std::setw(W) << "Feasible"
              << std::setw(W) << (our_feasible ? "SIM" : "NAO")
              << std::setw(W) << (vpr_feasible ? "SIM" : "NAO") << "\n";

    std::cout << std::setw(W) << "VTR routed OK"
              << std::setw(W) << "-"
              << std::setw(W) << (vpr_status.success() ? "SIM" : "NAO") << "\n";

    std::cout << std::string(60, '=') << "\n";

    vpr_free_all(arch, vpr_setup);
    return 0;
}
