#include <cmath>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>
#include <vector>

#include "globals.h"
#include "vpr_api.h"
#include "vpr_context/loader.h"
#include "ilp/ilp_router.h"

static int round_up_even(int n) {
    return (n % 2 == 0) ? n : n + 1;
}

// Deriva o nome do circuito a partir do caminho do .blif: basename sem diretório
// e cortado no primeiro '.' (ex.: ".../mult_4x4.pre-vpr.blif" -> "mult_4x4").
static std::string circuit_name_from_blif(const char* blif_path) {
    std::string p(blif_path);
    size_t slash = p.find_last_of("/\\");
    std::string base = (slash == std::string::npos) ? p : p.substr(slash + 1);
    size_t dot = base.find('.');
    return (dot == std::string::npos) ? base : base.substr(0, dot);
}

// Reconstrói o RR graph em W fixo, roteia com o VTR e executa o ILP.
// O rebuild+reroute por rodada (mesmo padrão de main.cpp) garante que
// route_trees e rr_graph em g_vpr_ctx — consumidos por run_ilp_routing —
// fiquem consistentes com o W desta rodada.
static bool run_round(t_vpr_setup& vpr_setup, t_arch& arch,
                      const ClusteredNetlist& nlist, int W,
                      const std::string& circuit_name,
                      const std::string& w_label, int time_limit) {
    std::cout << "\n##############################################\n";
    std::cout << ">>> RODADA W=" << W << " (" << w_label << ")\n";
    std::cout << "##############################################\n";

    vpr_create_rr_graph(vpr_setup, arch, W, /*is_flat=*/false);
    vpr_setup.RouterOpts.doRouting           = STAGE_DO;
    vpr_setup.RouterOpts.fixed_channel_width = W;

    RouteStatus st = vpr_route_flow((const Netlist<>&)nlist, vpr_setup, arch, /*is_flat=*/false);
    if (!st.success()) {
        std::cerr << "Erro: VTR não roteou com W=" << W << " (inesperado, W >= min_W).\n";
        return false;
    }
    std::cout << "VTR roteou com sucesso (W=" << W << ").\n";
    std::cout << "Nós RR : " << g_vpr_ctx.device().rr_graph.num_nodes() << "\n";

    IlpRunConfig cfg;
    cfg.circuit_name = circuit_name;
    cfg.W            = W;
    cfg.w_label      = w_label;
    cfg.time_limit   = time_limit;
    cfg.output_base  = "output";
    run_ilp_routing(cfg);
    return true;
}

int main(int argc, const char* argv[]) {
    if (argc < 5) {
        std::cerr << "Uso: " << argv[0]
                  << " <arch.xml> <circuito.blif> <circuito.net> <circuito.place>"
                  << " [modo: both|min|1.3x] [tempo_limite_seg]\n";
        return 1;
    }

    // Modo de rodada: both (default), min (só min_W) ou 1.3x (só 1.3*min_W).
    enum class Mode { BOTH, MIN, W130 };
    Mode mode = Mode::BOTH;
    if (argc >= 6) {
        if (std::strcmp(argv[5], "min") == 0)        mode = Mode::MIN;
        else if (std::strcmp(argv[5], "1.3x") == 0)  mode = Mode::W130;
        else if (std::strcmp(argv[5], "both") == 0)  mode = Mode::BOTH;
        else {
            std::cerr << "Modo inválido '" << argv[5] << "'. Use: both|min|1.3x\n";
            return 1;
        }
    }

    // Tempo limite do CPLEX (segundos): argv[6] opcional, default 300.
    int time_limit = 300;
    if (argc >= 7) {
        time_limit = std::atoi(argv[6]);
        if (time_limit <= 0) {
            std::cerr << "Tempo limite inválido '" << argv[6]
                      << "'. Use um inteiro positivo de segundos.\n";
            return 1;
        }
    }

    std::string circuit_name = circuit_name_from_blif(argv[2]);

    t_options   options;
    t_vpr_setup vpr_setup;
    t_arch      arch;

    load_vpr_context(argv[1], argv[2], argv[3], argv[4],
                     options, vpr_setup, arch);

    const auto& nlist = g_vpr_ctx.clustering().clb_nlist;
    std::cout << "Blocos : " << nlist.blocks().size() << "\n";
    std::cout << "Nets   : " << nlist.nets().size()   << "\n";

    // PASSO 0 — descobrir o W mínimo via VTR (binary search), em vez de fixar
    // um W arbitrário. Assim cada circuito roda no W que o VTR considera viável.
    std::cout << "\n>>> Descobrindo W mínimo via VTR (binary search)...\n";
    vpr_setup.RouterOpts.doRouting           = STAGE_DO;
    vpr_setup.RouterOpts.fixed_channel_width = NO_FIXED_CHANNEL_WIDTH;

    RouteStatus disc = vpr_route_flow((const Netlist<>&)nlist, vpr_setup, arch, /*is_flat=*/false);
    int min_W = disc.chan_width();
    if (min_W <= 0) {
        std::cerr << "Erro: VTR não encontrou W mínimo válido (result=" << min_W << ").\n";
        return 1;
    }
    int W_130 = round_up_even((int)std::ceil(1.3 * min_W));
    std::cout << "\n==> W mínimo encontrado : " << min_W << "\n";
    std::cout << "==> W x1.3 (arred. par) : " << W_130 << "\n";

    // Cada rodada carrega seu W e o rótulo correspondente (w_min / w_1.3x),
    // usado para organizar o diretório de saída.
    std::vector<std::pair<int, std::string>> rounds;
    if (mode == Mode::BOTH)      rounds = {{min_W, "w_min"}, {W_130, "w_1.3x"}};
    else if (mode == Mode::MIN)  rounds = {{min_W, "w_min"}};
    else                         rounds = {{W_130, "w_1.3x"}};

    for (const auto& [W, w_label] : rounds) {
        if (!run_round(vpr_setup, arch, nlist, W, circuit_name, w_label, time_limit)) {
            vpr_free_all(arch, vpr_setup);
            return 1;
        }
    }

    vpr_free_all(arch, vpr_setup);
    return 0;
}
