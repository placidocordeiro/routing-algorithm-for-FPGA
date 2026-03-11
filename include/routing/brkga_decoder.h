#ifndef ROUTING_BRKGA_DECODER_H
#define ROUTING_BRKGA_DECODER_H

#include "netlist/types.h"
#include "routing/router.h"
#include "routing/types.h"
#include "validation/validate.h"
#include <vector>

// Variáveis globais para acesso dentro da matrixF (setadas antes da execução)
extern const RoutingGraph* g_graph;
extern const std::vector<Net>* g_nets;

// Decodificador de soluções (ordem das nets)
class RoutingSolutionDecoder {
public:
    double soludecode(int id, const std::vector<double>& chromosome,
        const std::vector<std::vector<double>>& F,
        int solucriterion) const;
};

// Decodificador de cenários (criticalidade)
class RoutingScenarioDecoder {
public:
    double scendecode(int id, const std::vector<double>& maxD) const;
};

std::vector<std::vector<double>> neighDist(int MAX_THREADS,
    const std::vector<std::vector<double>>& F);

// Função que calcula a matriz de fitness F (solu_p x scen_p)
std::vector<std::vector<double>> matrixF(int MAX_THREADS,
    const std::vector<std::vector<double>>& pop_solu,
    const std::vector<std::vector<double>>& pop_scen);

#endif
