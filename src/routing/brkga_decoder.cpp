#include "routing/brkga_decoder.h"
#include "brkga/BRKGA.h" // apenas para ter acesso a Population, mas não necessário aqui
#include <algorithm>
#include <cmath>
#include <numeric>
#include <omp.h>

// Definição das variáveis globais
const RoutingGraph* g_graph = nullptr;
const std::vector<Net>* g_nets = nullptr;

double RoutingSolutionDecoder::soludecode(int id, const std::vector<double>& /*chromosome*/,
    const std::vector<std::vector<double>>& F,
    int solucriterion) const
{
    int nscen = (int)F[0].size();
    double fitness = 0.0;

    if (solucriterion == 2) { // pessimista (pior caso)
        fitness = F[id][0];
        for (int s = 1; s < nscen; ++s)
            if (F[id][s] > fitness)
                fitness = F[id][s];
    } else if (solucriterion == 3) { // otimista (melhor caso)
        fitness = F[id][0];
        for (int s = 1; s < nscen; ++s)
            if (F[id][s] < fitness)
                fitness = F[id][s];
    } else { // padrão: Laplace (média)
        double sum = 0.0;
        for (int s = 0; s < nscen; ++s)
            sum += F[id][s];
        fitness = sum / nscen;
    }
    return fitness;
}

// -------------------------------------------------------------------
// RoutingScenarioDecoder
// -------------------------------------------------------------------
double RoutingScenarioDecoder::scendecode(int id, const std::vector<double>& maxD) const
{
    // Queremos maximizar a distância, então fitness = -distância
    return -maxD[id];
}

// -------------------------------------------------------------------
// matrixF: avalia cada par (solução, cenário)
// -------------------------------------------------------------------
std::vector<std::vector<double>> matrixF(int MAX_THREADS,
    const std::vector<std::vector<double>>& pop_solu,
    const std::vector<std::vector<double>>& pop_scen)
{
    unsigned solu_p = (unsigned)pop_solu.size();
    unsigned scen_p = (unsigned)pop_scen.size();
    std::vector<std::vector<double>> F(solu_p, std::vector<double>(scen_p, 0.0));

    // Verifica se as variáveis globais foram setadas
    if (!g_graph || !g_nets) {
        throw std::runtime_error("matrixF: g_graph e g_nets devem ser inicializados.");
    }

#pragma omp parallel for num_threads(MAX_THREADS) schedule(dynamic)
    for (int idx = 0; idx < (int)(solu_p * scen_p); ++idx) {
        unsigned i = idx / scen_p; // índice da solução
        unsigned j = idx % scen_p; // índice do cenário

        // Decodifica a solução i: ordem das nets
        std::vector<int> order(pop_solu[i].size());
        std::iota(order.begin(), order.end(), 0);
        std::sort(order.begin(), order.end(),
            [&](int a, int b) { return pop_solu[i][a] < pop_solu[i][b]; });

        // Decodifica o cenário j: criticalidade (assume scen_n = 1)
        float criticality = (float)pop_scen[j][0]; // valor entre 0 e 1

        // Roteia com essa ordem e criticalidade
        Router router;
        RoutingGraph local_graph = *g_graph; // cópia para reset de uso
        local_graph.resetUsage();

        auto routes = router.route(local_graph, *g_nets, order, criticality);

        // Calcula estatísticas
        auto stats = Validator::validateRouting(local_graph, routes, *g_nets);

        // Fitness = penalidade (menor é melhor)
        double cost = 10.0 * stats.unrouted_nets + 5.0 * stats.overused_nodes + 5.0 * stats.disconnected_paths;
        F[i][j] = cost;
    }

    return F;
}

std::vector<std::vector<double>> neighDist(int MAX_THREADS,
    const std::vector<std::vector<double>>& F)
{
    if (F.empty() || F[0].empty())
        return {};

    size_t n_scen = F[0].size();
    std::vector<std::vector<double>> dist(n_scen, std::vector<double>(2, 0.0)); // 2 axes: 0-> how bad; 1-> how good

    std::vector<std::pair<double, int>> projectB(n_scen, std::make_pair(9999999999.9, 0));
    std::vector<std::pair<double, int>> projectW(n_scen, std::make_pair(-9999999999.9, 0));

    // Find best and worst projections
    for (size_t i = 0; i < F.size(); i++) {
        for (size_t j = 0; j < F[i].size(); j++) {
            if (F[i][j] < projectB[j].first) {
                projectB[j].first = F[i][j];
                projectB[j].second = j;
            }
            if (F[i][j] > projectW[j].first) {
                projectW[j].first = F[i][j];
                projectW[j].second = j;
            }
        }
    }

    std::sort(projectB.begin(), projectB.end());
    std::sort(projectW.begin(), projectW.end());

    // Ensure theoretical extremes are kept in the same positions (0 and 1)
    if (projectB[0].second != 1 && n_scen > 1) {
        for (size_t i = 1; i < projectB.size(); i++) {
            if (projectB[i].second == 1) {
                std::swap(projectB[0], projectB[i]);
                break;
            }
        }
    }

    if (projectW[n_scen - 1].second != 0 && n_scen > 1) {
        for (size_t i = projectW.size() - 2; i > 0; i--) {
            if (projectW[i].second == 0) {
                std::swap(projectW[n_scen - 1], projectW[i]);
                break;
            }
        }
    }

    // Calculate distance
    double BigM = 0.0;
    for (size_t i = 1; i < n_scen - 1; i++) {
        dist[projectB[i].second][0] = std::abs(projectB[i - 1].first - projectB[i].first) * std::abs(projectB[i + 1].first - projectB[i].first);
        dist[projectW[i].second][1] = std::abs(projectW[i - 1].first - projectW[i].first) * std::abs(projectW[i + 1].first - projectW[i].first);
        if (dist[projectB[i].second][0] > BigM) {
            BigM = dist[projectB[i].second][0];
        }
        if (dist[projectW[i].second][1] > BigM) {
            BigM = dist[projectW[i].second][1];
        }
    }

    // Extremes get higher distance
    if (n_scen > 0) {
        dist[projectB[0].second][0] = BigM + 1;
        dist[projectB[n_scen - 1].second][0] = BigM + 1;
        dist[projectW[0].second][1] = BigM + 1;
        dist[projectW[n_scen - 1].second][1] = BigM + 1;
    }

    return dist;
}
