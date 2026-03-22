#pragma once

#include "vpr_api.h"

// Inicializa todo o contexto VPR a partir dos arquivos de entrada.
// Após chamar esta função, g_vpr_ctx estará populado com
// netlist, device grid, RRGraph e placement prontos para uso.
void load_vpr_context(
    const char* arch_file,
    const char* blif_file,
    const char* net_file,
    const char* place_file,
    t_options&    options,
    t_vpr_setup&  vpr_setup,
    t_arch&       arch
);