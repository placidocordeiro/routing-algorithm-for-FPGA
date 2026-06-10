#pragma once

// Monta e resolve o modelo ILP de roteamento (CPLEX/Concert) usando os dados
// já carregados em g_vpr_ctx: RRGraph como grafo base e net_rr_terminals como
// sources/sinks de cada net.
// Requer que vpr_route_flow já tenha sido executado (net_rr_terminals populado).
void run_ilp_routing();
