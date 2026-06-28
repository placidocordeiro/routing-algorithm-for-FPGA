#include "ilp/ilp_router.h"

#include <ilcplex/ilocplex.h>

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <queue>
#include <streambuf>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "globals.h"
#include "net_delay.h"
#include "route_common.h"
#include "route_tree.h"
#include "routing/route_bridge.h"
#include "vpr_net_pins_matrix.h"

namespace {

// streambuf que encaminha cada escrita para dois destinos (ex.: cout + arquivo).
// Permite que o relatório do ILP apareça no console e seja persistido ao mesmo tempo.
class TeeBuf : public std::streambuf {
public:
    TeeBuf(std::streambuf* a, std::streambuf* b) : a_(a), b_(b) {}

protected:
    int overflow(int c) override {
        if (c == EOF) return !EOF;
        int r1 = a_ ? a_->sputc((char)c) : c;
        int r2 = b_ ? b_->sputc((char)c) : c;
        return (r1 == EOF || r2 == EOF) ? EOF : c;
    }
    int sync() override {
        int r1 = a_ ? a_->pubsync() : 0;
        int r2 = b_ ? b_->pubsync() : 0;
        return (r1 == 0 && r2 == 0) ? 0 : -1;
    }

private:
    std::streambuf* a_;
    std::streambuf* b_;
};

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
void print_delay_comparison(std::ostream& out,
                            const std::vector<NetData>& nets,
                            const NetPinsMatrix<float>& vtr_delay,
                            const NetPinsMatrix<float>& ilp_delay) {
    out << "\n========== DELAY VTR x ILP ==========\n";
    out << std::left << std::setw(24) << "Net"
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

        out << std::left << std::setw(24) << net.name
            << std::setw(16) << net_vtr_max * 1e9f
            << std::setw(16) << net_ilp_max * 1e9f
            << (net_vtr_max > 0 ? net_ilp_max / net_vtr_max : 0.0f) << "\n";
    }

    out << "-------------------------------------\n";
    out << "Max delay (ns)  : VTR " << vtr_max * 1e9f
        << " | ILP " << ilp_max * 1e9f
        << " | razao " << (vtr_max > 0 ? ilp_max / vtr_max : 0.0f) << "\n";
    out << "Soma sinks (ns) : VTR " << vtr_sum * 1e9
        << " | ILP " << ilp_sum * 1e9
        << " | razao " << (vtr_sum > 0 ? ilp_sum / vtr_sum : 0.0) << "\n";
    if (n_sinks > 0) {
        out << "Media sink (ns) : VTR " << (vtr_sum / n_sinks) * 1e9
            << " | ILP " << (ilp_sum / n_sinks) * 1e9 << "\n";
    }
    out << "=====================================\n";
}

// Procura um ciclo no subgrafo induzido por `used` (arestas com origem e destino
// ambos em `used`), iniciando a busca a partir de `start`. DFS com cores
// branco(0)/cinza(1)/preto(2): uma aresta para um nó cinza fecha um ciclo.
// Retorna a sequência de nós do ciclo (vazio se não houver).
std::vector<size_t> find_cycle_in_used(size_t start,
                                       const std::unordered_set<size_t>& used) {
    const auto& rr_graph = g_vpr_ctx.device().rr_graph;

    std::unordered_map<size_t, int>    color;   // 0=branco, 1=cinza, 2=preto
    std::unordered_map<size_t, size_t> parent;
    std::vector<size_t>                stack;    // pilha explícita (nó, prox. aresta)
    std::vector<t_edge_size>           edge_idx;

    stack.push_back(start);
    edge_idx.push_back(0);
    color[start] = 1;

    while (!stack.empty()) {
        size_t      node = stack.back();
        RRNodeId    n(node);
        t_edge_size ne = rr_graph.num_edges(n);
        bool        descended = false;

        for (t_edge_size& i = edge_idx.back(); i < ne; ) {
            size_t sink = size_t(rr_graph.edge_sink_node(n, i));
            i++;
            if (!used.count(sink)) continue;
            int c = color.count(sink) ? color[sink] : 0;
            if (c == 1) {
                // Aresta de retorno: reconstrói o ciclo node -> ... -> sink.
                std::vector<size_t> cycle;
                cycle.push_back(sink);
                for (size_t w = node; w != sink; w = parent[w]) cycle.push_back(w);
                cycle.push_back(sink);
                std::reverse(cycle.begin(), cycle.end());
                return cycle;
            }
            if (c == 0) {
                color[sink]  = 1;
                parent[sink] = node;
                stack.push_back(sink);
                edge_idx.push_back(0);
                descended = true;
                break;
            }
        }
        if (!descended) {
            color[node] = 2;
            stack.pop_back();
            edge_idx.pop_back();
        }
    }
    return {};
}

// Diagnóstico opt-in (ILP_DIAG): comprova que nets desconectadas se sustentam por
// ciclos de custo (base_cost) ~zero desconectados do source. Não altera o modelo.
void diagnose_disconnected(std::ostream&                out,
                           const std::vector<NetData>&  nets,
                           IloCplex&                    cplex,
                           const std::vector<IloBoolVarArray>& x,
                           const std::vector<NetBfs>&   net_bfs,
                           const std::vector<std::vector<size_t>>& preds,
                           size_t                       num_nodes) {
    const auto& rr_graph = g_vpr_ctx.device().rr_graph;
    const size_t k = nets.size();

    out << "\n========== DIAGNÓSTICO ILP_DIAG ==========\n";

    size_t total_disc = 0, total_orphans = 0, orphans_zero_cost = 0;

    for (size_t i = 0; i < k; i++) {
        size_t reached = count_reachable_sinks(nets[i], net_bfs[i]);
        if (reached == nets[i].sinks.size()) continue;
        total_disc++;

        std::unordered_set<size_t> used;
        for (size_t v = 0; v < num_nodes; v++) {
            if (cplex.getValue(x[i][v]) > 0.5) used.insert(v);
        }
        const auto& visited = net_bfs[i].visited;

        out << "\nNet " << nets[i].name << " (source " << nets[i].source
            << "): " << used.size() << " nós usados, "
            << visited.size() << " alcançáveis do source\n";

        // (1) Nós órfãos: usados mas não alcançáveis do source.
        out << "  Órfãos (used \\ visited):\n";
        for (size_t v : used) {
            if (visited.count(v)) continue;
            total_orphans++;
            float bc = get_single_rr_cong_base_cost(RRNodeId(v));
            if (bc == 0.0f) orphans_zero_cost++;
            out << "    node " << v
                << "  tipo " << rr_graph.node_type_string(RRNodeId(v))
                << "  base_cost " << bc
                << "  cap " << rr_graph.node_capacity(RRNodeId(v))
                << "  preds-em-used {";
            bool first = true;
            for (size_t u : preds[v]) {
                if (used.count(u)) {
                    out << (first ? "" : ",") << u;
                    first = false;
                }
            }
            out << "}\n";
        }

        // (2) Ciclo no subgrafo órfão. Procura a partir de cada sink não alcançado.
        for (size_t t : nets[i].sinks) {
            if (visited.count(t)) continue;
            std::vector<size_t> cycle = find_cycle_in_used(t, used);
            if (cycle.empty()) {
                out << "  Sink " << t
                    << ": nenhum ciclo encontrado a partir dele.\n";
                continue;
            }
            float cycle_cost = 0.0f;
            out << "  Ciclo sustentando sink " << t << ": ";
            for (size_t j = 0; j < cycle.size(); j++) {
                out << cycle[j] << (j + 1 < cycle.size() ? " -> " : "");
                if (j + 1 < cycle.size())
                    cycle_cost += get_single_rr_cong_base_cost(RRNodeId(cycle[j]));
            }
            out << "  | Σ base_cost = " << cycle_cost << "\n";
        }
    }

    out << "\n  Resumo: " << total_disc << " nets desconectadas, "
        << total_orphans << " nós órfãos ("
        << orphans_zero_cost << " com base_cost = 0)\n";
    out << "==========================================\n";
}

// Callback genérico do CPLEX, registrado por cima do modelo (add-on — não altera
// variáveis, restrições nem objetivo). Monitora o progresso do solve em modo
// read-only via contexto GLOBAL_PROGRESS (invocado por uma única thread, logo sem
// necessidade de mutex e sem afetar o resultado). Captura a timeline que o
// relatório pós-solve não conseguia medir: tempo até o 1º incumbent e número de
// melhorias de incumbent.
class IlpProgressCallback : public IloCplex::Callback::Function {
public:
    explicit IlpProgressCallback(std::chrono::steady_clock::time_point t0)
        : t0_(t0) {}

    void invoke(const IloCplex::Callback::Context& context) override {
        if (!context.inGlobalProgress()) return;
        ++num_calls_;

        // getIncumbentObjective() retorna IloInfinity (ou lança) enquanto não há
        // incumbent; tratamos ambos os casos para registrar o 1º de forma robusta.
        double inc = IloInfinity;
        try { inc = context.getIncumbentObjective(); } catch (IloException&) {}
        if (inc >= IloInfinity / 2.0) return;

        double now = std::chrono::duration<double>(
                         std::chrono::steady_clock::now() - t0_).count();
        if (t_first_incumbent_ < 0.0) {
            t_first_incumbent_ = now;
            best_incumbent_    = inc;
            ++num_incumbents_;
        } else if (inc < best_incumbent_ - 1e-9) {
            best_incumbent_ = inc;
            ++num_incumbents_;
        }
    }

    double t_first_incumbent() const { return t_first_incumbent_; }
    long   num_incumbents()    const { return num_incumbents_; }
    long   num_calls()         const { return num_calls_; }

private:
    std::chrono::steady_clock::time_point t0_;
    double t_first_incumbent_ = -1.0;
    double best_incumbent_    = 0.0;
    long   num_incumbents_    = 0;
    long   num_calls_         = 0;
};

} // namespace

void run_ilp_routing(const IlpRunConfig& cfg) {
    const auto& rr_graph = g_vpr_ctx.device().rr_graph;
    const auto& nlist    = g_vpr_ctx.clustering().clb_nlist;
    const size_t num_nodes = rr_graph.num_nodes();

    // Saída persistente: output/<circuito>/<time_limit>/<w_label>/resultado.txt.
    // O relatório é "tee-ado" para o console e para este arquivo simultaneamente.
    std::filesystem::path out_dir = std::filesystem::path(cfg.output_base) /
                                    cfg.circuit_name /
                                    std::to_string(cfg.time_limit) /
                                    cfg.w_label;
    std::error_code ec;
    std::filesystem::create_directories(out_dir, ec);
    if (ec) {
        std::cerr << "Aviso: não foi possível criar " << out_dir
                  << " (" << ec.message() << "); relatório só no console.\n";
    }
    std::ofstream file(out_dir / "resultado.txt", std::ios::trunc);
    TeeBuf tee(std::cout.rdbuf(), file.is_open() ? file.rdbuf() : nullptr);
    std::ostream out(&tee);

    out << "===================================\n";
    out << "Circuito   : " << cfg.circuit_name << "\n";
    out << "W (canal)  : " << cfg.W << " (" << cfg.w_label << ")\n";
    out << "Time limit : " << cfg.time_limit << " s\n";
    out << "===================================\n";

    // Delay do roteamento VTR — route_trees ainda intactas do vpr_route_flow.
    NetPinsMatrix<float> vtr_delay = make_net_pins_matrix<float>((const Netlist<>&)nlist);
    load_net_delay_from_routing((const Netlist<>&)nlist, vtr_delay);

    std::vector<NetData> nets = collect_nets();
    const size_t k = nets.size();

    out << "\n>>> ILP: " << k << " nets, " << num_nodes << " nós RR ("
        << k * num_nodes << " variáveis binárias)\n";

    // Lista de predecessores: preds[v] = {u : (u,v) ∈ E}
    std::vector<std::vector<size_t>> preds(num_nodes);
    for (RRNodeId node : rr_graph.nodes()) {
        for (t_edge_size i = 0; i < rr_graph.num_edges(node); i++) {
            preds[size_t(rr_graph.edge_sink_node(node, i))].push_back(size_t(node));
        }
    }

    // Índice global de arestas dirigidas, para as variáveis de fluxo.
    // edges[e] = {tail, head}; out_edges[v]/in_edges[v] = índices em `edges`.
    struct Edge { size_t tail, head; };
    std::vector<Edge>                edges;
    std::vector<std::vector<size_t>> out_edges(num_nodes), in_edges(num_nodes);
    for (RRNodeId node : rr_graph.nodes()) {
        size_t u = size_t(node);
        for (t_edge_size i = 0; i < rr_graph.num_edges(node); i++) {
            size_t w = size_t(rr_graph.edge_sink_node(node, i));
            size_t e = edges.size();
            edges.push_back({u, w});
            out_edges[u].push_back(e);
            in_edges[w].push_back(e);
        }
    }
    const size_t num_edges = edges.size();
    out << ">>> ILP: " << num_edges << " arestas RR ("
        << k * num_edges << " variáveis de fluxo)\n";

    // Tempo de build do modelo (construção + warm start), medido separado do
    // solve para a tabela de escalabilidade — o comparativo científico usa só o
    // tempo de solve, mas o tempo de build faz parte da história de escala.
    auto t_build0 = std::chrono::steady_clock::now();

    IloEnv env;
    try {
        IloModel model(env);

        // x[i][v] ∈ {0,1} — nó v usado pela net i
        std::vector<IloBoolVarArray> x;
        x.reserve(k);
        for (size_t i = 0; i < k; i++) {
            x.emplace_back(env, (IloInt)num_nodes);
        }

        // f[i][e] ∈ [0, D_i] — fluxo single-commodity na aresta e para a net i.
        // D_i = nº de sinks: source emite D_i unidades, cada sink consome 1.
        std::vector<IloNumVarArray> f;
        f.reserve(k);
        for (size_t i = 0; i < k; i++) {
            IloNum cap_flow = (IloNum)nets[i].sinks.size();
            f.emplace_back(env, (IloInt)num_edges, 0.0, cap_flow);
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

        // 2. Conectividade via fluxo single-commodity. A restrição de predecessor
        //    anterior (x[i][v] ≤ Σ preds) era necessária mas não suficiente: admitia
        //    ciclos desconectados do source sustentando sinks. O fluxo proíbe isso —
        //    um ciclo isolado não recebe fluxo, logo não entrega a unidade do sink.
        for (size_t i = 0; i < k; i++) {
            const IloNum D = (IloNum)nets[i].sinks.size();

            std::unordered_set<size_t> sink_set(nets[i].sinks.begin(),
                                                nets[i].sinks.end());

            // 2a. Conservação: (saída) − (entrada) = oferta − demanda em cada nó.
            for (size_t v = 0; v < num_nodes; v++) {
                IloExpr balance(env);
                for (size_t e : out_edges[v]) balance += f[i][e];
                for (size_t e : in_edges[v])  balance -= f[i][e];

                IloNum rhs = 0.0;
                if (v == nets[i].source)   rhs = D;       // source emite D_i
                else if (sink_set.count(v)) rhs = -1.0;    // cada sink consome 1
                model.add(balance == rhs);
                balance.end();
            }

            // 2b. Acoplamento fluxo↔nó: aresta só conduz fluxo se ambos os
            //     extremos estão usados (x = 1).
            for (size_t e = 0; e < num_edges; e++) {
                model.add(f[i][e] <= D * x[i][edges[e].tail]);
                model.add(f[i][e] <= D * x[i][edges[e].head]);
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
        cplex.setParam(IloCplex::Param::TimeLimit, cfg.time_limit);

        // Warm start: a solução do VTR (route_trees) como MIPStart completo.
        // Setamos tanto x quanto f — com fluxo, um MIPStart só com x seria
        // incompleto e o CPLEX poderia descartá-lo. O fluxo da árvore VTR é
        // reconstruído contando, por aresta, quantos sinks há na subárvore abaixo
        // dela (= soma das demandas que passam pela aresta no sentido source→sink).
        {
            const auto& route_ctx = g_vpr_ctx.routing();
            IloNumVarArray start_vars(env);
            IloNumArray    start_vals(env);
            size_t nets_with_start = 0;

            // Mapa (tail,head) -> índice de aresta, para casar arestas da route_tree.
            auto find_edge = [&](size_t tail, size_t head) -> long long {
                for (size_t e : out_edges[tail])
                    if (edges[e].head == head) return (long long)e;
                return -1;
            };

            for (size_t i = 0; i < k; i++) {
                ParentNetId pid{size_t(nets[i].id)};
                if (!route_ctx.route_trees[pid]) continue;
                nets_with_start++;

                std::unordered_set<size_t> used_vtr;
                // Parent de cada nó na árvore VTR (para walkback de cada sink).
                std::unordered_map<size_t, size_t> tree_parent;
                for (const RouteTreeNode& node : route_ctx.route_trees[pid]->all_nodes()) {
                    used_vtr.insert(size_t(node.inode));
                    if (node.parent())
                        tree_parent[size_t(node.inode)] = size_t(node.parent()->inode);
                }

                // Fluxo por aresta: +1 em cada aresta do caminho source→sink, por sink.
                std::vector<double> flow(num_edges, 0.0);
                for (size_t t : nets[i].sinks) {
                    size_t node = t;
                    while (node != nets[i].source && tree_parent.count(node)) {
                        size_t par = tree_parent[node];
                        long long e = find_edge(par, node);
                        if (e >= 0) flow[(size_t)e] += 1.0;
                        node = par;
                    }
                }

                for (size_t v = 0; v < num_nodes; v++) {
                    start_vars.add(x[i][v]);
                    start_vals.add(used_vtr.count(v) ? 1.0 : 0.0);
                }
                for (size_t e = 0; e < num_edges; e++) {
                    start_vars.add(f[i][e]);
                    start_vals.add(flow[e]);
                }
            }

            if (nets_with_start == k) {
                cplex.addMIPStart(start_vars, start_vals);
                out << ">>> Warm start: solução VTR fornecida como MIPStart ("
                    << nets_with_start << "/" << k << " nets)\n";
            } else {
                out << ">>> Warm start ignorado: apenas " << nets_with_start
                    << "/" << k << " nets têm route_tree do VTR\n";
            }
            start_vars.end();
            start_vals.end();
        }

        auto t0 = std::chrono::steady_clock::now();
        double build_time = std::chrono::duration<double>(t0 - t_build0).count();

        // Callback add-on (liga/desliga por env ILP_CALLBACK=off, default on).
        // Read-only; usado para A/B (ON vs OFF) sem mexer no modelo.
        const char* cb_env = std::getenv("ILP_CALLBACK");
        bool use_callback = !(cb_env && std::string(cb_env) == "off");
        IlpProgressCallback cb(t0);
        if (use_callback) {
            cplex.use(&cb, IloCplex::Callback::Context::Id::GlobalProgress);
        }

        bool solved = cplex.solve();
        auto t1 = std::chrono::steady_clock::now();
        double solve_time = std::chrono::duration<double>(t1 - t0).count();

        out << "\n========== RESULTADO ILP ==========\n";
        out << "Status CPLEX : " << cplex.getStatus() << "\n";
        out << "Tempo build  : " << build_time << " s\n";
        out << "Tempo solve  : " << solve_time << " s\n";

        if (!solved) {
            out << "ILP não encontrou solução.\n";
            env.end();
            return;
        }

        out << "Custo total (objetivo, base_cost): " << cplex.getObjValue() << "\n";
        out << "Melhor bound (best obj value)    : " << cplex.getBestObjValue() << "\n";
        out << "Gap relativo (MIP)               : "
            << cplex.getMIPRelativeGap() * 100.0 << " %\n";
        out << "Nós B&B explorados               : " << cplex.getNnodes() << "\n";
        out << "Variáveis (x / f)                : "
            << (k * num_nodes) << " / " << (k * num_edges) << "\n";
        out << "Restrições (CPLEX getNrows)      : " << cplex.getNrows() << "\n";
        if (use_callback) {
            out << "Callback         : ON (progress)\n";
            out << "Tempo 1o incumbent: "
                << (cb.t_first_incumbent() >= 0.0
                        ? std::to_string(cb.t_first_incumbent()) + " s"
                        : "(nenhum)") << "\n";
            out << "Incumbents (melhorias): " << cb.num_incumbents() << "\n";
        } else {
            out << "Callback         : OFF\n";
        }
        out << "\n";

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

            out << "Net " << nets[i].name
                << ": " << used.size() << " nós usados, sinks alcançados "
                << reached << "/" << nets[i].sinks.size()
                << (ok ? "" : "  [DESCONECTADA]") << "\n";
        }
        out << "\nNets conectadas (BFS): " << nets_ok << "/" << k << "\n";
        out << "===================================\n";

        if (std::getenv("ILP_DIAG") && nets_ok != k) {
            diagnose_disconnected(out, nets, cplex, x, net_bfs, preds, num_nodes);
        }

        if (nets_ok != k) {
            out << "Nets desconectadas — comparação de delay abortada.\n";
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

        print_delay_comparison(out, nets, vtr_delay, ilp_delay);
    } catch (IloException& e) {
        std::cerr << "Erro CPLEX: " << e.getMessage() << "\n";
    }
    env.end();
}
