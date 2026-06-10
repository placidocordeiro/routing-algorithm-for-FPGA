#include "ilp/ilp_router.h"

#include <ilcplex/ilocplex.h>

#include <algorithm>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "globals.h"
#include "net_delay.h"
#include "route_common.h"
#include "route_tree.h"
#include "routing/steiner_router.h"
#include "vpr_net_pins_matrix.h"

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
// Grava parent_node/parent_edge para permitir o walkback até cada sink.
struct NetBfs {
    std::unordered_set<size_t>           visited;
    std::unordered_map<size_t, size_t>   parent_node;
    std::unordered_map<size_t, RREdgeId> parent_edge;
};

NetBfs bfs_selected_nodes(const NetData& net, const std::unordered_set<size_t>& used) {
    const auto& rr_graph = g_vpr_ctx.device().rr_graph;

    NetBfs bfs;
    std::queue<size_t> queue;
    queue.push(net.source);
    bfs.visited.insert(net.source);

    while (!queue.empty()) {
        RRNodeId node(queue.front());
        queue.pop();
        RREdgeId first_edge = rr_graph.node_first_edge(node);
        for (t_edge_size i = 0; i < rr_graph.num_edges(node); i++) {
            size_t sink = size_t(rr_graph.edge_sink_node(node, i));
            if (used.count(sink) && !bfs.visited.count(sink)) {
                bfs.visited.insert(sink);
                bfs.parent_node[sink] = size_t(node);
                bfs.parent_edge[sink] = RREdgeId(size_t(first_edge) + i);
                queue.push(sink);
            }
        }
    }
    return bfs;
}

size_t count_reachable_sinks(const NetData& net, const NetBfs& bfs) {
    size_t reached = 0;
    for (size_t t : net.sinks) {
        if (bfs.visited.count(t)) reached++;
    }
    return reached;
}

// Reconstrói o caminho [source, ..., sink] a partir dos parents do BFS,
// no mesmo formato que PathFinder::find_path produz.
routing::PathResult walkback_path(const NetBfs& bfs, size_t source, size_t sink) {
    routing::PathResult path;

    size_t node = sink;
    path.nodes.push_back(RRNodeId(node));
    while (node != source) {
        path.edges.push_back(bfs.parent_edge.at(node));
        node = bfs.parent_node.at(node);
        path.nodes.push_back(RRNodeId(node));
    }
    std::reverse(path.edges.begin(), path.edges.end());
    std::reverse(path.nodes.begin(), path.nodes.end());
    path.success = true;
    return path;
}

// Compara os delays Elmore (source→sink) dos dois roteamentos.
void print_delay_comparison(const std::vector<NetData>& nets,
                            const NetPinsMatrix<float>& vtr_delay,
                            const NetPinsMatrix<float>& ilp_delay) {
    std::cout << "\n========== DELAY VTR x ILP ==========\n";
    std::cout << std::left << std::setw(24) << "Net"
              << std::setw(16) << "VTR max (ns)"
              << std::setw(16) << "ILP max (ns)"
              << "ILP/VTR\n";

    float vtr_max = 0, ilp_max = 0;
    double vtr_sum = 0, ilp_sum = 0;
    size_t n_sinks = 0;

    for (const NetData& net : nets) {
        ParentNetId pid{size_t(net.id)};
        float net_vtr_max = 0, net_ilp_max = 0;
        for (size_t j = 1; j <= net.sinks.size(); j++) {
            net_vtr_max = std::max(net_vtr_max, vtr_delay[pid][j]);
            net_ilp_max = std::max(net_ilp_max, ilp_delay[pid][j]);
            vtr_sum += vtr_delay[pid][j];
            ilp_sum += ilp_delay[pid][j];
            n_sinks++;
        }
        vtr_max = std::max(vtr_max, net_vtr_max);
        ilp_max = std::max(ilp_max, net_ilp_max);

        std::cout << std::left << std::setw(24) << net.name
                  << std::setw(16) << net_vtr_max * 1e9f
                  << std::setw(16) << net_ilp_max * 1e9f
                  << (net_vtr_max > 0 ? net_ilp_max / net_vtr_max : 0.0f) << "\n";
    }

    std::cout << "-------------------------------------\n";
    std::cout << "Max delay (ns)  : VTR " << vtr_max * 1e9f
              << " | ILP " << ilp_max * 1e9f
              << " | razao " << (vtr_max > 0 ? ilp_max / vtr_max : 0.0f) << "\n";
    std::cout << "Soma sinks (ns) : VTR " << vtr_sum * 1e9
              << " | ILP " << ilp_sum * 1e9
              << " | razao " << (vtr_sum > 0 ? ilp_sum / vtr_sum : 0.0) << "\n";
    if (n_sinks > 0) {
        std::cout << "Media sink (ns) : VTR " << (vtr_sum / n_sinks) * 1e9
                  << " | ILP " << (ilp_sum / n_sinks) * 1e9 << "\n";
    }
    std::cout << "=====================================\n";
}

} // namespace

void run_ilp_routing() {
    const auto& rr_graph = g_vpr_ctx.device().rr_graph;
    const auto& nlist    = g_vpr_ctx.clustering().clb_nlist;
    const size_t num_nodes = rr_graph.num_nodes();

    // Delay do roteamento VTR — route_trees ainda intactas do vpr_route_flow.
    NetPinsMatrix<float> vtr_delay = make_net_pins_matrix<float>((const Netlist<>&)nlist);
    load_net_delay_from_routing((const Netlist<>&)nlist, vtr_delay);

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

        // Objetivo: min Σ_i Σ_v c_v · x[i][v], com c_v = base_cost(v) —
        // a componente estática do custo PathFinder (acc/pres são dinâmicas).
        IloExpr obj(env);
        for (size_t i = 0; i < k; i++) {
            for (size_t v = 0; v < num_nodes; v++) {
                obj += (IloNum)get_single_rr_cong_base_cost(RRNodeId(v)) * x[i][v];
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

        // Warm start: a solução do VTR (route_trees) como MIPStart completo.
        {
            const auto& route_ctx = g_vpr_ctx.routing();
            IloNumVarArray start_vars(env);
            IloNumArray    start_vals(env);
            size_t nets_with_start = 0;

            for (size_t i = 0; i < k; i++) {
                ParentNetId pid{size_t(nets[i].id)};
                if (!route_ctx.route_trees[pid]) continue;
                nets_with_start++;

                std::unordered_set<size_t> used_vtr;
                for (const RouteTreeNode& node : route_ctx.route_trees[pid]->all_nodes()) {
                    used_vtr.insert(size_t(node.inode));
                }
                for (size_t v = 0; v < num_nodes; v++) {
                    start_vars.add(x[i][v]);
                    start_vals.add(used_vtr.count(v) ? 1.0 : 0.0);
                }
            }

            if (nets_with_start == k) {
                cplex.addMIPStart(start_vars, start_vals);
                std::cout << ">>> Warm start: solução VTR fornecida como MIPStart ("
                          << nets_with_start << "/" << k << " nets)\n";
            } else {
                std::cout << ">>> Warm start ignorado: apenas " << nets_with_start
                          << "/" << k << " nets têm route_tree do VTR\n";
            }
            start_vars.end();
            start_vals.end();
        }

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

        std::cout << "Custo total (objetivo, base_cost): " << cplex.getObjValue() << "\n\n";

        // Extrair nós usados e verificar conectividade de cada net via BFS.
        std::vector<NetBfs> net_bfs(k);
        size_t nets_ok = 0;
        for (size_t i = 0; i < k; i++) {
            std::unordered_set<size_t> used;
            for (size_t v = 0; v < num_nodes; v++) {
                if (cplex.getValue(x[i][v]) > 0.5) used.insert(v);
            }
            net_bfs[i] = bfs_selected_nodes(nets[i], used);
            size_t reached = count_reachable_sinks(nets[i], net_bfs[i]);
            bool ok = (reached == nets[i].sinks.size());
            if (ok) nets_ok++;

            std::cout << "Net " << nets[i].name
                      << ": " << used.size() << " nós usados, sinks alcançados "
                      << reached << "/" << nets[i].sinks.size()
                      << (ok ? "" : "  [DESCONECTADA]") << "\n";
        }
        std::cout << "\nNets conectadas (BFS): " << nets_ok << "/" << k << "\n";
        std::cout << "===================================\n";

        if (nets_ok != k) {
            std::cout << "Nets desconectadas — comparação de delay abortada.\n";
            env.end();
            return;
        }

        // Reconstruir route_trees a partir da solução ILP: teardown seletivo
        // (nets fora do ILP mantêm a tree do VTR para o load_net_delay) e
        // espelhamento dos caminhos via mirror_path_to_vtr.
        auto& route_ctx = g_vpr_ctx.mutable_routing();
        for (size_t i = 0; i < k; i++) {
            ParentNetId pid{size_t(nets[i].id)};
            if (route_ctx.route_trees[pid]) {
                pathfinder_update_cost_from_route_tree(
                    route_ctx.route_trees[pid]->root(), -1);
                route_ctx.route_trees[pid] = vtr::nullopt;
            }
            for (size_t j = 0; j < nets[i].sinks.size(); j++) {
                routing::PathResult path =
                    walkback_path(net_bfs[i], nets[i].source, nets[i].sinks[j]);
                routing::mirror_path_to_vtr(pid, /*sink_pin_index=*/(int)(j + 1), path);
            }
        }

        NetPinsMatrix<float> ilp_delay = make_net_pins_matrix<float>((const Netlist<>&)nlist);
        load_net_delay_from_routing((const Netlist<>&)nlist, ilp_delay);

        print_delay_comparison(nets, vtr_delay, ilp_delay);
    } catch (IloException& e) {
        std::cerr << "Erro CPLEX: " << e.getMessage() << "\n";
    }
    env.end();
}
