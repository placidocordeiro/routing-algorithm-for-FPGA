#pragma once

#include <string>

// Configuração de uma rodada do ILP: identifica o circuito/W/rótulo da rodada,
// o time limit do CPLEX e a base do diretório de saída. Usada para persistir o
// relatório em output/<circuito>/<time_limit>/<w_label>/resultado.txt.
struct IlpRunConfig {
    std::string circuit_name;   // ex.: "mult_4x4"
    int         W;              // largura de canal desta rodada
    std::string w_label;        // "w_min" ou "w_1.3x"
    int         time_limit;     // segundos (CPLEX TimeLimit)
    std::string output_base;    // base do diretório de saída (ex.: "output")
};

// Monta e resolve o modelo ILP de roteamento (CPLEX/Concert) usando os dados
// já carregados em g_vpr_ctx: RRGraph como grafo base e net_rr_terminals como
// sources/sinks de cada net.
// Requer que vpr_route_flow já tenha sido executado (net_rr_terminals populado).
void run_ilp_routing(const IlpRunConfig& cfg);
