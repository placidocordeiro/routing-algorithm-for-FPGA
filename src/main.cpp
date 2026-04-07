#include <iostream>
#include <iomanip>
#include <sstream>
#include <chrono>
#include <cmath>
#include <string>
#include "globals.h"
#include "vpr_context/loader.h"
#include "routing/steiner_router.h"
#include "route_common.h"
#include "route_utils.h"
#include "router_stats.h"
#include "check_route.h"
#include "vpr_api.h"

struct RoundMetrics {
    int    chan_width        = 0;
    double steiner_time     = 0.0;
    double vtr_time         = 0.0;
    bool   steiner_feasible = false;
    bool   vtr_feasible     = false;
    bool   vtr_success      = false;
    size_t steiner_wl_used  = 0;
    size_t vtr_wl_used      = 0;
    size_t wl_avail         = 0;
    float  steiner_wl_ratio = 0.0f;
    float  vtr_wl_ratio     = 0.0f;
    int    nets_routed      = 0;
    int    total_nets       = 0;
};

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

// Arredonda para o inteiro par mais próximo (acima)
static int round_up_even(int n) {
    return (n % 2 == 0) ? n : n + 1;
}

static RoundMetrics run_round(const ClusteredNetlist& nlist,
                              t_vpr_setup& vpr_setup,
                              const t_arch& arch,
                              int chan_width) {
    RoundMetrics m;
    m.chan_width = chan_width;

    // O RR graph já deve estar em chan_width antes de chamar esta função.
    const auto& rr_graph = g_vpr_ctx.device().rr_graph;

    // ── Steiner Router ────────────────────────────────────────────────────────
    std::cout << "\n>>> [W=" << chan_width << "] Executando Steiner Router...\n";
    routing::SteinerRouter router(1.0f);

    auto t0 = std::chrono::steady_clock::now();
    auto result = router.route(nlist, rr_graph);
    auto t1 = std::chrono::steady_clock::now();
    m.steiner_time  = std::chrono::duration<double>(t1 - t0).count();
    m.nets_routed   = result.nets_routed;
    m.total_nets    = result.total_nets;
    result.print_summary();

    collect_metrics(nlist,
                    m.steiner_feasible,
                    m.steiner_wl_used,
                    m.wl_avail,
                    m.steiner_wl_ratio);

    // ── VTR Router (W fixo) ───────────────────────────────────────────────────
    std::cout << "\n>>> [W=" << chan_width << "] Executando VTR Router nativo (fixed W)...\n";
    vpr_setup.RouterOpts.doRouting          = STAGE_DO;
    vpr_setup.RouterOpts.fixed_channel_width = chan_width;

    auto t2 = std::chrono::steady_clock::now();
    RouteStatus vtr_status = vpr_route_flow((const Netlist<>&)nlist, vpr_setup, arch, /*is_flat=*/false);
    auto t3 = std::chrono::steady_clock::now();
    m.vtr_time    = std::chrono::duration<double>(t3 - t2).count();
    m.vtr_success = vtr_status.success();

    collect_metrics(nlist,
                    m.vtr_feasible,
                    m.vtr_wl_used,
                    m.wl_avail,
                    m.vtr_wl_ratio);

    return m;
}

static void print_table(const RoundMetrics& r_min, const RoundMetrics& r_130) {
    const int CW = 22;
    const std::string sep(5 * CW, '=');
    const std::string dash(5 * CW, '-');

    auto col = [&](auto v) -> std::string {
        if constexpr (std::is_same_v<decltype(v), bool>)
            return v ? "SIM" : "NAO";
        else
            return std::to_string(v);
    };

    std::cout << "\n" << sep << "\n"
              << "  COMPARAÇÃO DE ROTEAMENTO\n"
              << sep << "\n";

    std::cout << std::left
              << std::setw(CW) << ""
              << std::setw(CW) << ("Steiner W=" + std::to_string(r_min.chan_width))
              << std::setw(CW) << ("VTR    W=" + std::to_string(r_min.chan_width))
              << std::setw(CW) << ("Steiner W=" + std::to_string(r_130.chan_width))
              << std::setw(CW) << ("VTR    W=" + std::to_string(r_130.chan_width))
              << "\n" << dash << "\n";

    auto row = [&](const char* label, std::string v1, std::string v2, std::string v3, std::string v4) {
        std::cout << std::setw(CW) << label
                  << std::setw(CW) << v1
                  << std::setw(CW) << v2
                  << std::setw(CW) << v3
                  << std::setw(CW) << v4 << "\n";
    };

    auto fstr = [](double v, int prec = 3) {
        std::ostringstream ss;
        ss << std::fixed << std::setprecision(prec) << v;
        return ss.str();
    };

    row("Tempo (s)",
        fstr(r_min.steiner_time), fstr(r_min.vtr_time),
        fstr(r_130.steiner_time), fstr(r_130.vtr_time));

    row("Nets roteadas",
        std::to_string(r_min.nets_routed) + "/" + std::to_string(r_min.total_nets), "-",
        std::to_string(r_130.nets_routed) + "/" + std::to_string(r_130.total_nets), "-");

    row("WL usado",
        std::to_string(r_min.steiner_wl_used), std::to_string(r_min.vtr_wl_used),
        std::to_string(r_130.steiner_wl_used), std::to_string(r_130.vtr_wl_used));

    row("WL disponivel",
        std::to_string(r_min.wl_avail), std::to_string(r_min.wl_avail),
        std::to_string(r_130.wl_avail), std::to_string(r_130.wl_avail));

    row("Utilizacao (%)",
        fstr(r_min.steiner_wl_ratio * 100.0f, 1), fstr(r_min.vtr_wl_ratio * 100.0f, 1),
        fstr(r_130.steiner_wl_ratio * 100.0f, 1), fstr(r_130.vtr_wl_ratio * 100.0f, 1));

    row("Feasible",
        (r_min.steiner_feasible ? "SIM" : "NAO"), (r_min.vtr_feasible ? "SIM" : "NAO"),
        (r_130.steiner_feasible ? "SIM" : "NAO"), (r_130.vtr_feasible ? "SIM" : "NAO"));

    row("VTR routed OK",
        "-", (r_min.vtr_success ? "SIM" : "NAO"),
        "-", (r_130.vtr_success ? "SIM" : "NAO"));

    std::cout << sep << "\n";
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

    const auto& nlist = g_vpr_ctx.clustering().clb_nlist;

    std::cout << "Blocos : " << nlist.blocks().size() << "\n";
    std::cout << "Nets   : " << nlist.nets().size()   << "\n";
    std::cout << "Nós RR : " << g_vpr_ctx.device().rr_graph.nodes().size() << "\n\n";

    // =========================================================================
    // PASSO 0 — Rodar VTR sem W fixo para descobrir min_W (binary search)
    // =========================================================================
    std::cout << ">>> Descobrindo W mínimo via VTR (binary search)...\n";
    vpr_setup.RouterOpts.doRouting           = STAGE_DO;
    vpr_setup.RouterOpts.fixed_channel_width = NO_FIXED_CHANNEL_WIDTH;

    RouteStatus vpr_discovery = vpr_route_flow((const Netlist<>&)nlist, vpr_setup, arch, /*is_flat=*/false);
    int min_W = vpr_discovery.chan_width();

    if (min_W <= 0) {
        std::cerr << "Erro: VTR não encontrou W mínimo válido (result=" << min_W << ").\n";
        return 1;
    }

    int W_130 = round_up_even((int)std::ceil(1.3 * min_W));

    std::cout << "\n==> W mínimo encontrado : " << min_W << "\n";
    std::cout << "==> W x1.3 (arred. par) : " << W_130 << "\n\n";

    // =========================================================================
    // RODADA 1 — W = min_W
    // Após binary search o RR graph já está em min_W; garante com create_rr_graph.
    // =========================================================================
    vpr_create_rr_graph(vpr_setup, arch, min_W, /*is_flat=*/false);
    RoundMetrics m_min = run_round(nlist, vpr_setup, arch, min_W);

    // =========================================================================
    // RODADA 2 — W = 1.3 * min_W
    // Reconstrói RR graph para o novo W antes de rodar os roteadores.
    // =========================================================================
    vpr_create_rr_graph(vpr_setup, arch, W_130, /*is_flat=*/false);
    RoundMetrics m_130 = run_round(nlist, vpr_setup, arch, W_130);

    // =========================================================================
    // TABELA COMPARATIVA
    // =========================================================================
    print_table(m_min, m_130);

    vpr_free_all(arch, vpr_setup);
    return 0;
}