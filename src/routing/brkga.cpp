#include "routing/brkga.h"
#include <algorithm>
#include <numeric>

BRKGA::BRKGA(const BRKGAParams& params)
    : params_(params)
    , rng_(std::random_device {}())
{
}

void BRKGA::initialize_population(int num_nets)
{
    std::uniform_real_distribution<float> dist(0.0f, 1.0f);
    population_.clear();

    for (int i = 0; i < params_.population_size; ++i) {
        Individual ind;
        ind.keys.resize(num_nets);
        for (auto& k : ind.keys)
            k = dist(rng_);
        population_.push_back(ind);
    }
}

std::vector<int> BRKGA::decode(const Individual& ind) const
{
    std::vector<int> order(ind.keys.size());
    std::iota(order.begin(), order.end(), 0);

    std::sort(order.begin(), order.end(),
        [&](int a, int b) {
            return ind.keys[a] < ind.keys[b];
        });

    return order;
}

void BRKGA::evaluate_population(const RoutingGraph& graph,
    const std::vector<Net>& nets)
{
    Router router;

    for (auto& ind : population_) {
        RoutingGraph local_graph = graph;
        local_graph.resetUsage();

        auto order = decode(ind);
        auto routes = router.route(local_graph, nets, order);

        auto stats = Validator::validateRouting(local_graph, routes, nets);

        ind.fitness = 10.0f * stats.unrouted_nets + 5.0f * stats.overused_nodes + 5.0f * stats.disconnected_paths;
    }
}

Individual BRKGA::crossover(const Individual& elite,
    const Individual& non_elite)
{
    std::uniform_real_distribution<float> dist(0.0f, 1.0f);
    Individual child;
    child.keys.resize(elite.keys.size());

    for (size_t i = 0; i < elite.keys.size(); ++i) {
        child.keys[i] = (dist(rng_) < params_.inheritance_prob)
            ? elite.keys[i]
            : non_elite.keys[i];
    }
    return child;
}

bool BRKGA::run(const RoutingGraph& graph,
    const std::vector<Net>& nets,
    std::vector<int>& best_order)
{

    initialize_population(nets.size());

    for (int gen = 0; gen < params_.max_generations; ++gen) {
        evaluate_population(graph, nets);

        std::sort(population_.begin(), population_.end(),
            [](const Individual& a, const Individual& b) {
                return a.fitness < b.fitness;
            });

        if (population_[0].fitness == 0.0f) {
            best_order = decode(population_[0]);
            return true;
        }

        std::vector<Individual> next_pop;
        for (int i = 0; i < params_.elite_size; ++i)
            next_pop.push_back(population_[i]);

        std::uniform_int_distribution<int> elite_dist(0, params_.elite_size - 1);
        std::uniform_int_distribution<int> non_elite_dist(
            params_.elite_size, params_.population_size - 1);

        while ((int)next_pop.size() < params_.population_size) {
            auto& e = population_[elite_dist(rng_)];
            auto& n = population_[non_elite_dist(rng_)];
            next_pop.push_back(crossover(e, n));
        }

        population_ = next_pop;
    }

    best_order = decode(population_[0]);
    return false;
}
