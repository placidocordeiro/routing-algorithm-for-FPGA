#include "ilp/ilp_router.h"

#include <ilcplex/ilocplex.h>

#include <chrono>
#include <iostream>
#include <queue>
#include <unordered_set>
#include <vector>

#include "globals.h"

namespace {

struct NetData {
    ClusterNetId         id;
    std::string          name;
    size_t               source;
    std::vector<size_t>  sinks;
};

// Coleta as nets roteáveis com seus terminais SOURCE/SINK no RRGraph.
std::vector<NetData> collect_nets() {
    const auto& nlist       = g_vpr_ctx.clustering().clb_nlist;
    const auto& routing_ctx = g_vpr_ctx.routing();

    std::vector<NetData> nets;
    for (ClusterNetId net_id : nlist.nets()) {
        if (nlist.net_is_ignored(net_id)) continue;  // clocks/globals não são roteadas

        ParentNetId pid{size_t(net_id)};
        const auto& terminals = routing_ctx.net_rr_terminals[pid];
        if (terminals.size() < 2) continue;

        NetData net;
        net.id     = net_id;
        net.name   = nlist.net_name(net_id);
        net.source = size_t(terminals[0]);
        for (size_t j = 1; j < terminals.size(); j++) {
            net.sinks.push_back(size_t(terminals[j]));
        }
        nets.push_back(std::move(net));
    }
    return nets;
}

// BFS no RRGraph restrito aos nós selecionados pela net, a partir do source.
// Retorna quantos sinks da net foram de fato alcançados.
size_t count_reachable_sinks(const NetData& net, const std::unordered_set<size_t>& used) {
    const auto& rr_graph = g_vpr_ctx.device().rr_graph;

    std::unordered_set<size_t> visited;
    std::queue<size_t> queue;
    queue.push(net.source);
    visited.insert(net.source);

    while (!queue.empty()) {
        RRNodeId node(queue.front());
        queue.pop();
        for (t_edge_size i = 0; i < rr_graph.num_edges(node); i++) {
            size_t sink = size_t(rr_graph.edge_sink_node(node, i));
            if (used.count(sink) && !visited.count(sink)) {
                visited.insert(sink);
                queue.push(sink);
            }
        }
    }

    size_t reached = 0;
    for (size_t t : net.sinks) {
        if (visited.count(t)) reached++;
    }
    return reached;
}

} // namespace

void run_ilp_routing() {
    const auto& rr_graph = g_vpr_ctx.device().rr_graph;
    const size_t num_nodes = rr_graph.num_nodes();

    std::vector<NetData> nets = collect_nets();
    const size_t k = nets.size();

    std::cout << "\n>>> ILP: " << k << " nets, " << num_nodes << " nós RR ("
              << k * num_nodes << " variáveis binárias)\n";

    // Lista de predecessores: preds[v] = {u : (u,v) ∈ E}
    std::vector<std::vector<size_t>> preds(num_nodes);
    for (RRNodeId node : rr_graph.nodes()) {
        for (t_edge_size i = 0; i < rr_graph.num_edges(node); i++) {
            preds[size_t(rr_graph.edge_sink_node(node, i))].push_back(size_t(node));
        }
    }

    IloEnv env;
    try {
        IloModel model(env);

        // x[i][v] ∈ {0,1} — nó v usado pela net i
        std::vector<IloBoolVarArray> x;
        x.reserve(k);
        for (size_t i = 0; i < k; i++) {
            x.emplace_back(env, (IloInt)num_nodes);
        }

        // Objetivo: min Σ_i Σ_v c_v · x[i][v], com c_v = 1
        IloExpr obj(env);
        for (size_t i = 0; i < k; i++) {
            for (size_t v = 0; v < num_nodes; v++) {
                obj += x[i][v];
            }
        }
        model.add(IloMinimize(env, obj));
        obj.end();

        // 1. Capacidade de vértice: Σ_i x[i][v] ≤ cap(v)
        //    cap(v) vem do RRGraph: SOURCE/SINK podem ter capacidade > 1;
        //    fixar 1 nesses nós tornaria o modelo infeasible sem necessidade.
        for (size_t v = 0; v < num_nodes; v++) {
            IloExpr sum(env);
            for (size_t i = 0; i < k; i++) {
                sum += x[i][v];
            }
            model.add(sum <= (IloInt)rr_graph.node_capacity(RRNodeId(v)));
            sum.end();
        }

        // 2. Conectividade: x[i][v] ≤ Σ_{u ∈ preds(v)} x[i][u], ∀ v ≠ source(i)
        //    (soma vazia força x[i][v] = 0 em nós sem predecessores)
        for (size_t i = 0; i < k; i++) {
            for (size_t v = 0; v < num_nodes; v++) {
                if (v == nets[i].source) continue;
                IloExpr sum(env);
                for (size_t u : preds[v]) {
                    sum += x[i][u];
                }
                model.add(x[i][v] <= sum);
                sum.end();
            }
        }

        // 3. Source e sinks obrigatórios
        for (size_t i = 0; i < k; i++) {
            model.add(x[i][nets[i].source] == 1);
            for (size_t t : nets[i].sinks) {
                model.add(x[i][t] == 1);
            }
        }

        IloCplex cplex(model);
        cplex.setParam(IloCplex::Param::TimeLimit, 300);

        auto t0 = std::chrono::steady_clock::now();
        bool solved = cplex.solve();
        auto t1 = std::chrono::steady_clock::now();
        double solve_time = std::chrono::duration<double>(t1 - t0).count();

        std::cout << "\n========== RESULTADO ILP ==========\n";
        std::cout << "Status CPLEX : " << cplex.getStatus() << "\n";
        std::cout << "Tempo solve  : " << solve_time << " s\n";

        if (!solved) {
            std::cout << "ILP não encontrou solução.\n";
            env.end();
            return;
        }

        std::cout << "WL total (objetivo): " << cplex.getObjValue() << "\n\n";

        size_t nets_ok = 0;
        for (size_t i = 0; i < k; i++) {
            std::unordered_set<size_t> used;
            for (size_t v = 0; v < num_nodes; v++) {
                if (cplex.getValue(x[i][v]) > 0.5) used.insert(v);
            }
            size_t reached = count_reachable_sinks(nets[i], used);
            bool ok = (reached == nets[i].sinks.size());
            if (ok) nets_ok++;

            std::cout << "Net " << nets[i].name
                      << ": " << used.size() << " nós usados, sinks alcançados "
                      << reached << "/" << nets[i].sinks.size()
                      << (ok ? "" : "  [DESCONECTADA]") << "\n";
        }
        std::cout << "\nNets conectadas (BFS): " << nets_ok << "/" << k << "\n";
        std::cout << "===================================\n";
    } catch (IloException& e) {
        std::cerr << "Erro CPLEX: " << e.getMessage() << "\n";
    }
    env.end();
}
