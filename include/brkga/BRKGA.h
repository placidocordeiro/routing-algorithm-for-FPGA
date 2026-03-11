#ifndef BRKGA_H
#define BRKGA_H

#include "Population.h"
#include <algorithm>
#include <exception>
#include <omp.h>
#include <stdexcept>

template <class SoluDecoder, class ScenDecoder, class RNG>
class BRKGA {
public:
    BRKGA(unsigned solu_n, unsigned solu_p, double solu_pe, double solu_pm, double solu_rhoe,
        unsigned scen_n, unsigned scen_p, double scen_pe, double scen_pm, double scen_rhoe,
        const std::vector<double> extremeworst, const std::vector<double> extremebest,
        const SoluDecoder& refSoluDecoder, const ScenDecoder& refScenDecoder,
        RNG& refRNG, const int solucriterion = 1, unsigned K = 1, unsigned MAX_THREADS = 1);

    ~BRKGA();

    /**
     * Resets all populations with brand new keys
     */
    void reset();

    /**
     * Evolve the current populations following the guidelines of BRKGAs
     * @param generations number of generations (must be even and nonzero)
     * @param J interval to exchange elite chromosomes (must be even; 0 ==> no synchronization)
     * @param M number of elite chromosomes to select from each population in order to exchange
     */
    void evolve(unsigned generations = 1);

    /**
     * Exchange elite-solutions between the populations
     * @param M number of elite chromosomes to select from each population
     */
    void exchangeEliteSolu(unsigned M);
    void exchangeEliteScen(unsigned M);

    /**
     * Returns the current population
     */
    const Population& getPopulationSolu(unsigned k = 0) const;
    const Population& getPopulationScen(unsigned k = 0) const;

    /**
     * Returns the chromosome with best fitness so far among all populations
     */
    const std::vector<double>& getBestChromosomeSolu() const;
    const std::vector<double>& getBestChromosomeScen() const;

    /**
     * Returns the best fitness found so far among all populations
     */
    double getBestFitnessSolu() const;
    double getBestFitnessScen() const;

    // Return copies to the internal parameters:
    unsigned getN_solu() const;
    unsigned getP_solu() const;
    unsigned getPe_solu() const;
    unsigned getPm_solu() const;
    unsigned getPo_solu() const;
    double getRhoe_solu() const;
    unsigned getN_scen() const;
    unsigned getP_scen() const;
    unsigned getPe_scen() const;
    unsigned getPm_scen() const;
    unsigned getPo_scen() const;
    double getRhoe_scen() const;
    unsigned getK() const;
    unsigned getMAX_THREADS() const;

private:
    // Hyperparameters:
    const unsigned solu_n; // number of genes in the solution chromosome
    const unsigned solu_p; // number of elements in the solution population
    const unsigned solu_pe; // number of elite items in the solution population
    const unsigned solu_pm; // number of mutants introduced at each generation into the solution population
    const double solu_rhoe; // probability that a solution offspring inherits the allele of its elite parent
    const unsigned scen_n; // number of genes in the scenario chromosome
    const unsigned scen_p; // number of elements in the scenario population
    const unsigned scen_pe; // number of elite items in the scenario population
    const unsigned scen_pm; // number of mutants introduced at each generation into the scenario population
    const double scen_rhoe; // probability that a scenario offspring inherits the allele of its elite parent

    const int solucriter; // criterion for solution performance
    const std::vector<double> extremebest; // theoretical best scenario
    const std::vector<double> extremeworst; // theoretical worst scenario

    // Templates:
    RNG& refRNG; // reference to the random number generator
    const SoluDecoder& refSoluDecoder; // reference to the solution Decoder
    const ScenDecoder& refScenDecoder; // reference to the scenario Decoder

    // Parallel populations parameters:
    const unsigned K; // number of independent parallel populations
    const unsigned MAX_THREADS; // number of threads for parallel decoding

    // Data:
    std::vector<Population*> solu_previous; // previous solution populations
    std::vector<Population*> solu_current; // current solution populations
    std::vector<Population*> scen_previous; // previous scenario populations
    std::vector<Population*> scen_current; // current scenario populations

    // Local operations:
    void initialize(const unsigned i); // initialize current population 'i' with random keys
    void evolution(Population& solucurr, Population& solunext, Population& scencurr, Population& scennext);
    bool isRepeated(const std::vector<double>& chrA, const std::vector<double>& chrB) const;
};

template <class SoluDecoder, class ScenDecoder, class RNG>
BRKGA<SoluDecoder, ScenDecoder, RNG>::BRKGA(unsigned _solu_n, unsigned _solu_p, double _solu_pe, double _solu_pm,
    double _solu_rhoe, unsigned _scen_n, unsigned _scen_p, double _scen_pe,
    double _scen_pm, double _scen_rhoe,
    const std::vector<double> _extremeworst, const std::vector<double> _extremebest,
    const SoluDecoder& soldecoder, const ScenDecoder& scendecoder,
    RNG& rng, int solucriterion, unsigned _K, unsigned MAX)
    : solu_n(_solu_n)
    , solu_p(_solu_p)
    , solu_pe(unsigned(_solu_pe * solu_p))
    , solu_pm(unsigned(_solu_pm * solu_p))
    , solu_rhoe(_solu_rhoe)
    , scen_n(_scen_n)
    , scen_p(_scen_p)
    , scen_pe(unsigned(_scen_pe * scen_p))
    , scen_pm(unsigned(_scen_pm * scen_p))
    , scen_rhoe(_scen_rhoe)
    , extremeworst(_extremeworst)
    , extremebest(_extremebest)
    , refRNG(rng)
    , solucriter(solucriterion)
    , refSoluDecoder(soldecoder)
    , refScenDecoder(scendecoder)
    , K(_K)
    , MAX_THREADS(MAX)
    , solu_previous(K, 0)
    , solu_current(K, 0)
    , scen_previous(K, 0)
    , scen_current(K, 0)
{

    // Error check:
    using std::range_error;
    if (solu_n == 0) {
        throw range_error("Chromosome size (solution) equals zero.");
    }
    if (solu_p == 0) {
        throw range_error("Population size (solution) equals zero.");
    }
    if (solu_pe == 0) {
        throw range_error("Elite-set size (solution) equals zero.");
    }
    if (solu_pe > solu_p) {
        throw range_error("Elite-set size (solution) greater than population size (pe > p).");
    }
    if (solu_pm > solu_p) {
        throw range_error("Mutant-set size (pm) (solution) greater than population size (p).");
    }
    if (solu_pe + solu_pm > solu_p) {
        throw range_error("elite + mutant (solution) sets greater than population size (p).");
    }
    if (scen_n == 0) {
        throw range_error("Chromosome size (scenario) equals zero.");
    }
    if (scen_p == 0) {
        throw range_error("Population size (scenario) equals zero.");
    }
    if (scen_pe == 0) {
        throw range_error("Elite-set size (scenario) equals zero.");
    }
    if (scen_pe > scen_p) {
        throw range_error("Elite-set size (scenario) greater than population size (pe > p).");
    }
    if (scen_pm > scen_p) {
        throw range_error("Mutant-set size (pm) (scenario) greater than population size (p).");
    }
    if (scen_pe + scen_pm > scen_p) {
        throw range_error("elite + mutant (scenario) sets greater than population size (p).");
    }
    if (solucriter != 1 && solucriter != 2 && solucriter != 3) {
        throw range_error("solution performance criterion must be 1, 2 or 3");
    }
    if (K == 0) {
        throw range_error("Number of parallel populations cannot be zero.");
    }

    // Initialize and decode each chromosome of the current population, then copy to previous:
    for (unsigned i = 0; i < K; ++i) {
        // Allocate:
        solu_current[i] = new Population(solu_n, solu_p);
        scen_current[i] = new Population(scen_n, scen_p);

        // Initialize:
        initialize(i);

        // Then just copy to previous:
        solu_previous[i] = new Population(*solu_current[i]);
        scen_previous[i] = new Population(*scen_current[i]);
    }
}

template <class SoluDecoder, class ScenDecoder, class RNG>
BRKGA<SoluDecoder, ScenDecoder, RNG>::~BRKGA()
{
    for (unsigned i = 0; i < K; ++i) {
        delete solu_current[i];
        delete solu_previous[i];
        delete scen_current[i];
        delete scen_previous[i];
    }
}

template <class SoluDecoder, class ScenDecoder, class RNG>
const Population& BRKGA<SoluDecoder, ScenDecoder, RNG>::getPopulationSolu(unsigned k) const
{
    return (*solu_current[k]);
}

template <class SoluDecoder, class ScenDecoder, class RNG>
const Population& BRKGA<SoluDecoder, ScenDecoder, RNG>::getPopulationScen(unsigned k) const
{
    return (*scen_current[k]);
}

template <class SoluDecoder, class ScenDecoder, class RNG>
double BRKGA<SoluDecoder, ScenDecoder, RNG>::getBestFitnessSolu() const
{
    double best = solu_current[0]->fitness[0].first;
    for (unsigned i = 1; i < K; ++i) {
        if (solu_current[i]->fitness[0].first < best) {
            best = solu_current[i]->fitness[0].first;
        }
    }

    return best;
}

template <class SoluDecoder, class ScenDecoder, class RNG>
double BRKGA<SoluDecoder, ScenDecoder, RNG>::getBestFitnessScen() const
{
    double best = scen_current[0]->fitness[0].first;
    for (unsigned i = 1; i < K; ++i) {
        if (scen_current[i]->fitness[0].first < best) {
            best = scen_current[i]->fitness[0].first;
        }
    }

    return best;
}

template <class SoluDecoder, class ScenDecoder, class RNG>
const std::vector<double>& BRKGA<SoluDecoder, ScenDecoder, RNG>::getBestChromosomeSolu() const
{
    unsigned bestK = 0;
    for (unsigned i = 1; i < K; ++i) {
        if (solu_current[i]->getBestFitness() < solu_current[bestK]->getBestFitness()) {
            bestK = i;
        }
    }

    return solu_current[bestK]->getChromosome(0);
}

template <class SoluDecoder, class ScenDecoder, class RNG>
const std::vector<double>& BRKGA<SoluDecoder, ScenDecoder, RNG>::getBestChromosomeScen() const
{
    unsigned bestK = 0;
    for (unsigned i = 1; i < K; ++i) {
        if (scen_current[i]->getBestFitness() < scen_current[bestK]->getBestFitness()) {
            bestK = i;
        }
    }

    return scen_current[bestK]->getChromosome(0);
}

template <class SoluDecoder, class ScenDecoder, class RNG>
void BRKGA<SoluDecoder, ScenDecoder, RNG>::reset()
{
    for (unsigned i = 0; i < K; ++i) {
        initialize(i);
    }
}

template <class SoluDecoder, class ScenDecoder, class RNG>
void BRKGA<SoluDecoder, ScenDecoder, RNG>::evolve(unsigned generations)
{
    if (generations == 0) {
        throw std::range_error("Cannot evolve for 0 generations.");
    }

    for (unsigned i = 0; i < generations; ++i) {
        for (unsigned j = 0; j < K; ++j) {
            evolution(*solu_current[j], *solu_previous[j], *scen_current[j], *scen_previous[j]); // First evolve the population (curr, next)
            std::swap(solu_current[j], solu_previous[j]); // Update (prev = curr; curr = prev == next)
            std::swap(scen_current[j], scen_previous[j]); // Update (prev = curr; curr = prev == next)
        }
    }
}

template <class SoluDecoder, class ScenDecoder, class RNG>
void BRKGA<SoluDecoder, ScenDecoder, RNG>::exchangeEliteSolu(unsigned M)
{
    if (M == 0 || M >= solu_p) {
        throw std::range_error("M cannot be zero or >= p.");
    }

    for (unsigned i = 0; i < K; ++i) {
        // Population i will receive some elite members from each Population j below:
        unsigned dest = solu_p - 1; // Last chromosome of i (will be updated below)
        for (unsigned j = 0; j < K; ++j) {
            if (j == i) {
                continue;
            }

            // Copy the M best of Population j into Population i:
            for (unsigned m = 0; m < M; ++m) {
                // Copy the m-th best of Population j into the 'dest'-th position of Population i:
                const std::vector<double>& bestOfJ = solu_current[j]->getChromosome(m);

                std::copy(bestOfJ.begin(), bestOfJ.end(), solu_current[i]->getChromosome(dest).begin());

                solu_current[i]->fitness[dest].first = solu_current[j]->fitness[m].first;

                --dest;
            }
        }
    }

    for (int j = 0; j < int(K); ++j) {
        solu_current[j]->sortFitness();
    }
}

template <class SoluDecoder, class ScenDecoder, class RNG>
void BRKGA<SoluDecoder, ScenDecoder, RNG>::exchangeEliteScen(unsigned M)
{
    if (M == 0 || M >= scen_p) {
        throw std::range_error("M cannot be zero or >= p.");
    }

    for (unsigned i = 0; i < K; ++i) {
        // Population i will receive some elite members from each Population j below:
        unsigned dest = scen_p - 1; // Last chromosome of i (will be updated below)
        for (unsigned j = 0; j < K; ++j) {
            if (j == i) {
                continue;
            }

            // Copy the M best of Population j into Population i:
            for (unsigned m = 0; m < M; ++m) {
                // Copy the m-th best of Population j into the 'dest'-th position of Population i:
                const std::vector<double>& bestOfJ = scen_current[j]->getChromosome(m + 2); // alterado para +2 para evitar copiar TWS e TBS sempre

                std::copy(bestOfJ.begin(), bestOfJ.end(), scen_current[i]->getChromosome(dest).begin());

                scen_current[i]->fitness[dest].first = scen_current[j]->fitness[m + 2].first; // alterado para +2 para evitar copiar TWS e TBS sempre

                --dest;
            }
        }
    }

    for (int j = 0; j < int(K); ++j) {
        scen_current[j]->sortFitness();
    }
}

template <class SoluDecoder, class ScenDecoder, class RNG>
inline void BRKGA<SoluDecoder, ScenDecoder, RNG>::initialize(const unsigned i)
{

    // generate initial solution population randomly
    for (unsigned j = 0; j < solu_p; ++j) {
        for (unsigned k = 0; k < solu_n; ++k) {
            (*solu_current[i])(j, k) = refRNG.rand();
        }
    }

    // generate initial scenario population
    if (extremebest.size() > 0 && extremeworst.size() > 0) {
        // create theoretically extreme scenarios
        (*scen_current[i])(0) = extremeworst;
        (*scen_current[i])(1) = extremebest;

        // generate the remaining scenarios randomly
        for (unsigned j = 2; j < scen_p; ++j) {
            for (unsigned k = 0; k < scen_n; ++k) {
                (*scen_current[i])(j, k) = refRNG.rand();
            }
        }
    } else {
        // generate all scenarios randomly
        for (unsigned j = 0; j < scen_p; ++j) {
            for (unsigned k = 0; k < scen_n; ++k) {
                (*scen_current[i])(j, k) = refRNG.rand();
            }
        }
    }

    // Decode:
    std::vector<std::vector<double>> F = matrixF(MAX_THREADS, solu_current[i]->getChromosomes(), scen_current[i]->getChromosomes());
    std::vector<std::vector<double>> D = neighDist(MAX_THREADS, F);
    std::vector<double> maxD(D.size(), 0.0);
    for (int k = 0; k < D.size(); k++) {
        maxD[k] = *std::max_element(D[k].begin(), D[k].end());
    }

#ifdef _OPENMP
#pragma omp parallel for num_threads(MAX_THREADS)
#endif
    for (int j = 0; j < int(solu_p); ++j) {
        solu_current[i]->setFitness(j, refSoluDecoder.soludecode(j, (*solu_current[i])(j), F, solucriter));
    }

#ifdef _OPENMP
#pragma omp parallel for num_threads(MAX_THREADS)
#endif
    for (int j = 0; j < int(scen_p); ++j) {
        scen_current[i]->setFitness(j, refScenDecoder.scendecode(j, maxD));
    }

    // Sort:
    solu_current[i]->sortFitness();
    scen_current[i]->sortFitness();
}

template <class SoluDecoder, class ScenDecoder, class RNG>
inline void BRKGA<SoluDecoder, ScenDecoder, RNG>::evolution(Population& solucurr, Population& solunext, Population& scencurr, Population& scennext)
{

    unsigned i = 0; // Iterate chromosome by chromosome
    unsigned j = 0; // Iterate allele by allele

    // Solution population
    // Copy elite
    while (i < solu_pe) {
        for (j = 0; j < solu_n; ++j) {
            solunext(i, j) = solucurr(solucurr.fitness[i].second, j);
        }

        ++i;
    }
    // Cross-over
    while (i < solu_p - solu_pm) {
        const unsigned eliteParent = (refRNG.randInt(solu_pe - 1));
        const unsigned noneliteParent = solu_pe + (refRNG.randInt(solu_p - solu_pe - 1));
        for (j = 0; j < solu_n; ++j) {
            const unsigned sourceParent = ((refRNG.rand() < solu_rhoe) ? eliteParent : noneliteParent);
            solunext(i, j) = solucurr(solucurr.fitness[sourceParent].second, j);
        }
        ++i;
    }

    // Mutation
    while (i < solu_p) {
        for (j = 0; j < solu_n; ++j) {
            solunext(i, j) = refRNG.rand();
        }
        ++i;
    }

    // Scenario population

    i = 0;
    j = 0;

    // Copy elite
    while (i < scen_pe) {
        for (j = 0; j < scen_n; ++j) {
            scennext(i, j) = scencurr(scencurr.fitness[i].second, j);
        }
        ++i;
    }

    // Cross-over
    while (i < scen_p - scen_pm) {
        const unsigned eliteParent = (refRNG.randInt(scen_pe - 1));
        const unsigned noneliteParent = scen_pe + (refRNG.randInt(scen_p - scen_pe - 1));
        for (j = 0; j < scen_n; ++j) {
            const unsigned sourceParent = ((refRNG.rand() < scen_rhoe) ? eliteParent : noneliteParent);
            scennext(i, j) = scencurr(scencurr.fitness[sourceParent].second, j);
        }
        ++i;
    }

    // Mutation
    while (i < scen_p) {
        for (j = 0; j < scen_n; ++j) {
            scennext(i, j) = refRNG.rand();
        }
        ++i;
    }

    // Fitness calculation
    std::vector<std::vector<double>> F = matrixF(MAX_THREADS, solunext.getChromosomes(), scennext.getChromosomes());
    std::vector<std::vector<double>> D = neighDist(MAX_THREADS, F);
    std::vector<double> maxD(D.size(), 0.0);
    for (int k = 0; k < D.size(); k++) {
        maxD[k] = *std::max_element(D[k].begin(), D[k].end());
    }
#ifdef _OPENMP
#pragma omp parallel for num_threads(MAX_THREADS)
#endif
    for (int i = 0; i < int(solu_p); ++i) {
        solunext.setFitness(i, refSoluDecoder.soludecode(i, solunext.population[i], F, solucriter));
    }
#ifdef _OPENMP
#pragma omp parallel for num_threads(MAX_THREADS)
#endif
    for (int i = 0; i < int(scen_p); ++i) {
        scennext.setFitness(i, refScenDecoder.scendecode(i, maxD));
    }

    // Sort
    solunext.sortFitness();
    scennext.sortFitness();
}

template <class SoluDecoder, class ScenDecoder, class RNG>
unsigned BRKGA<SoluDecoder, ScenDecoder, RNG>::getN_solu() const { return solu_n; }

template <class SoluDecoder, class ScenDecoder, class RNG>
unsigned BRKGA<SoluDecoder, ScenDecoder, RNG>::getN_scen() const { return scen_n; }

template <class SoluDecoder, class ScenDecoder, class RNG>
unsigned BRKGA<SoluDecoder, ScenDecoder, RNG>::getP_solu() const { return solu_p; }

template <class SoluDecoder, class ScenDecoder, class RNG>
unsigned BRKGA<SoluDecoder, ScenDecoder, RNG>::getP_scen() const { return scen_p; }

template <class SoluDecoder, class ScenDecoder, class RNG>
unsigned BRKGA<SoluDecoder, ScenDecoder, RNG>::getPe_solu() const { return solu_pe; }

template <class SoluDecoder, class ScenDecoder, class RNG>
unsigned BRKGA<SoluDecoder, ScenDecoder, RNG>::getPe_scen() const { return scen_pe; }

template <class SoluDecoder, class ScenDecoder, class RNG>
unsigned BRKGA<SoluDecoder, ScenDecoder, RNG>::getPm_solu() const { return solu_pm; }

template <class SoluDecoder, class ScenDecoder, class RNG>
unsigned BRKGA<SoluDecoder, ScenDecoder, RNG>::getPm_scen() const { return scen_pm; }

template <class SoluDecoder, class ScenDecoder, class RNG>
unsigned BRKGA<SoluDecoder, ScenDecoder, RNG>::getPo_solu() const { return solu_p - solu_pe - solu_pm; }

template <class SoluDecoder, class ScenDecoder, class RNG>
unsigned BRKGA<SoluDecoder, ScenDecoder, RNG>::getPo_scen() const { return scen_p - scen_pe - scen_pm; }

template <class SoluDecoder, class ScenDecoder, class RNG>
double BRKGA<SoluDecoder, ScenDecoder, RNG>::getRhoe_solu() const { return solu_rhoe; }

template <class SoluDecoder, class ScenDecoder, class RNG>
double BRKGA<SoluDecoder, ScenDecoder, RNG>::getRhoe_scen() const { return scen_rhoe; }

template <class SoluDecoder, class ScenDecoder, class RNG>
unsigned BRKGA<SoluDecoder, ScenDecoder, RNG>::getK() const { return K; }

template <class SoluDecoder, class ScenDecoder, class RNG>
unsigned BRKGA<SoluDecoder, ScenDecoder, RNG>::getMAX_THREADS() const { return MAX_THREADS; }

#endif
