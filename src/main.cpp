#include <iostream>
#include "globals.h"
#include "vpr_context/loader.h"

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

    // Contextos prontos para uso
    auto& cluster_ctx = g_vpr_ctx.clustering();
    auto& device_ctx  = g_vpr_ctx.device();
    auto& place_ctx   = g_vpr_ctx.placement();

    const auto& nlist    = cluster_ctx.clb_nlist;
    const auto& rr_graph = device_ctx.rr_graph;

    std::cout << "Blocos : " << nlist.blocks().size() << "\n";
    std::cout << "Nets   : " << nlist.nets().size()   << "\n";
    std::cout << "Nós RR : " << rr_graph.num_nodes()  << "\n";

    // === o algoritmo de roteamento entra aqui ===

    vpr_free_all(arch, vpr_setup);
    return 0;
}