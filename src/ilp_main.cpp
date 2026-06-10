#include <cstdlib>
#include <iostream>

#include "globals.h"
#include "vpr_api.h"
#include "vpr_context/loader.h"
#include "ilp/ilp_router.h"

int main(int argc, const char* argv[]) {
    if (argc < 5) {
        std::cerr << "Uso: " << argv[0]
                  << " <arch.xml> <circuito.blif> <circuito.net> <circuito.place> [W=20]\n";
        return 1;
    }
    int chan_width = (argc >= 6) ? std::atoi(argv[5]) : 20;

    t_options   options;
    t_vpr_setup vpr_setup;
    t_arch      arch;

    load_vpr_context(argv[1], argv[2], argv[3], argv[4],
                     options, vpr_setup, arch);

    const auto& nlist = g_vpr_ctx.clustering().clb_nlist;
    std::cout << "Blocos : " << nlist.blocks().size() << "\n";
    std::cout << "Nets   : " << nlist.nets().size()   << "\n";

    // Roteia com o VTR em W fixo: popula net_rr_terminals e garante que o
    // mesmo RRGraph que o ILP vai usar admite solução.
    std::cout << "\n>>> Roteando com VTR (W=" << chan_width << ") para popular contexto...\n";
    vpr_setup.RouterOpts.doRouting           = STAGE_DO;
    vpr_setup.RouterOpts.fixed_channel_width = chan_width;

    RouteStatus status = vpr_route_flow((const Netlist<>&)nlist, vpr_setup, arch, /*is_flat=*/false);
    if (!status.success()) {
        std::cerr << "Erro: VTR não roteou com W=" << chan_width
                  << " — aumente W e tente de novo.\n";
        return 1;
    }
    std::cout << "VTR roteou com sucesso (W=" << chan_width << ").\n";
    std::cout << "Nós RR : " << g_vpr_ctx.device().rr_graph.num_nodes() << "\n";

    run_ilp_routing();

    vpr_free_all(arch, vpr_setup);
    return 0;
}
