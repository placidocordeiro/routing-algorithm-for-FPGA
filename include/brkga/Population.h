#ifndef POPULATION_H
#define POPULATION_H

#include <algorithm>
#include <exception>
#include <stdexcept>
#include <vector>

class Population {
    template <class SoluDecoder, class ScenDecoder, class RNG>
    friend class BRKGA;
    friend class SolutionDecoder;
    friend class ScenarioDecoder;

public:
    unsigned getN() const { return population[0].size(); }
    unsigned getP() const { return population.size(); }

    double getBestFitness() const { return getFitness(0); }

    double getFitness(unsigned i) const { return fitness[i].first; }

    const std::vector<double>& getChromosome(unsigned i) const
    {
        return population[fitness[i].second];
    }

    const std::vector<std::vector<double>>& getChromosomes() const
    {
        return population;
    }

private:
    Population(const Population& other)
        : population(other.population)
        , fitness(other.fitness)
    {
    }

    Population(unsigned n, unsigned p)
        : population(p, std::vector<double>(n, 0.0))
        , fitness(p)
    {
        if (p == 0) {
            throw std::range_error("Population size p cannot be zero.");
        }
        if (n == 0) {
            throw std::range_error("Chromosome size n cannot be zero.");
        }
    }

    ~Population() = default;

    std::vector<std::vector<double>> population;
    std::vector<std::pair<double, unsigned>> fitness;

    void sortFitness()
    {
        std::sort(fitness.begin(), fitness.end());
    }

    void setFitness(unsigned i, double f)
    {
        fitness[i].first = f;
        fitness[i].second = i;
    }

    std::vector<double>& getChromosome(unsigned i)
    {
        return population[fitness[i].second];
    }

    double& operator()(unsigned chromosome, unsigned allele)
    {
        return population[chromosome][allele];
    }

    std::vector<double>& operator()(unsigned chromosome)
    {
        return population[chromosome];
    }
};

#endif
