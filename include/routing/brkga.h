#ifndef ROUTING_BRKGA_H
#define ROUTING_BRKGA_H

#include "routing/router.h"
#include "routing/types.h"
#include "validation/validate.h"
#include <random>
#include <vector>

struct BRKGAParams {
    int population_size = 30;
    int elite_size = 5;
    int max_generations = 50;
    float inheritance_prob = 0.7f;
};

struct Individual {
    std::vector<float> keys; // random keys
    float fitness = 1e9f;
};

class BRKGA {
public:
    BRKGA(const BRKGAParams& params);

    // Executa o algoritmo e retorna a melhor ordem encontrada
    bool run(const RoutingGraph& graph, const std::vector<Net>& nets,
        std::vector<int>& best_order);

private:
    BRKGAParams params_;
    std::mt19937 rng_;

    std::vector<Individual> population_;

    void initialize_population(int num_nets);
    void evaluate_population(const RoutingGraph& graph,
        const std::vector<Net>& nets);
    std::vector<int> decode(const Individual& ind) const;
    Individual crossover(const Individual& elite, const Individual& non_elite);
};

#endif
