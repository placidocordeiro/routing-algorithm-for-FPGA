#include "ilp/ilp_router.h"

#include <ilcplex/ilocplex.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <queue>
#include <stdexcept>
#include <streambuf>
#include <string>
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

struct Edge {
    size_t tail;
    size_t head;
    size_t rr_edge_id;
    size_t switch_id;
    double delay_proxy_ns;
};

// Domínio esparso de uma net. Os índices de x/f são locais, mas nodes/edges
// guardam o índice correspondente no RRGraph global. A formulação permanece a
// mesma; variáveis fora deste subgrafo induzido são implicitamente fixadas em 0.
struct NetDomain {
    std::vector<size_t> nodes;
    std::vector<size_t> edges;
    bool fallback_full_graph = false;
};

struct VtrRouteData {
    std::unordered_set<size_t> used_nodes;
    std::unordered_map<size_t, size_t> parent;
    std::unordered_map<size_t, size_t> parent_edge;
    bool complete = false;
};

struct BoundingBox {
    int xmin = std::numeric_limits<int>::max();
    int ymin = std::numeric_limits<int>::max();
    int xmax = std::numeric_limits<int>::min();
    int ymax = std::numeric_limits<int>::min();

    bool valid() const { return xmin <= xmax && ymin <= ymax; }
};

size_t local_index(const std::vector<size_t>& sorted_ids, size_t global_id) {
    auto it = std::lower_bound(sorted_ids.begin(), sorted_ids.end(), global_id);
    if (it == sorted_ids.end() || *it != global_id)
        return std::numeric_limits<size_t>::max();
    return size_t(it - sorted_ids.begin());
}

bool env_enabled(const char* name, bool default_value) {
    const char* value = std::getenv(name);
    if (!value) return default_value;
    return std::string(value) != "off" && std::string(value) != "0"
           && std::string(value) != "false";
}

int bbox_margin_from_env(std::ostream& out) {
    constexpr int default_margin = 3;
    const char* value = std::getenv("ILP_BBOX_MARGIN");
    if (!value) return default_margin;

    char* end = nullptr;
    long parsed = std::strtol(value, &end, 10);
    if (end == value || *end != '\0' || parsed < 0
        || parsed > std::numeric_limits<int>::max()) {
        out << "Aviso: ILP_BBOX_MARGIN='" << value
            << "' inválido; usando " << default_margin << ".\n";
        return default_margin;
    }
    return int(parsed);
}

// Coleta as nets roteáveis com seus terminais SOURCE/SINK no RRGraph.
std::vector<NetData> collect_nets() {
    const auto& nlist       = g_vpr_ctx.clustering().clb_nlist;
    const auto& routing_ctx = g_vpr_ctx.routing();

    std::vector<NetData> nets;
    for (ClusterNetId net_id : nlist.nets()) {
        if (nlist.net_is_ignored(net_id)) continue;  // clocks/globals não são roteadas

        ParentNetId pid{static_cast<int>(size_t(net_id))};
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

void include_node_in_box(BoundingBox& box, size_t node_id) {
    const auto& rr_graph = g_vpr_ctx.device().rr_graph;
    RRNodeId node(node_id);
    box.xmin = std::min(box.xmin, int(rr_graph.node_xlow(node)));
    box.ymin = std::min(box.ymin, int(rr_graph.node_ylow(node)));
    box.xmax = std::max(box.xmax, int(rr_graph.node_xhigh(node)));
    box.ymax = std::max(box.ymax, int(rr_graph.node_yhigh(node)));
}

bool node_intersects_box(size_t node_id, const BoundingBox& box, int margin) {
    const auto& rr_graph = g_vpr_ctx.device().rr_graph;
    RRNodeId node(node_id);
    const long long m = margin;
    return static_cast<long long>(rr_graph.node_xhigh(node))
               >= static_cast<long long>(box.xmin) - m
           && static_cast<long long>(rr_graph.node_xlow(node))
                  <= static_cast<long long>(box.xmax) + m
           && static_cast<long long>(rr_graph.node_yhigh(node))
                  >= static_cast<long long>(box.ymin) - m
           && static_cast<long long>(rr_graph.node_ylow(node))
                  <= static_cast<long long>(box.ymax) + m;
}

std::vector<VtrRouteData> collect_vtr_routes(const std::vector<NetData>& nets,
                                             const std::vector<Edge>& edges,
                                             const std::vector<std::vector<size_t>>& out_edges) {
    const auto& route_ctx = g_vpr_ctx.routing();
    std::vector<VtrRouteData> routes(nets.size());

    auto find_edge = [&](size_t tail, size_t head, size_t switch_id) -> size_t {
        for (size_t edge_id : out_edges[tail]) {
            if (edges[edge_id].head == head
                && edges[edge_id].switch_id == switch_id) {
                return edge_id;
            }
        }
        return std::numeric_limits<size_t>::max();
    };

    for (size_t i = 0; i < nets.size(); i++) {
        ParentNetId pid{static_cast<int>(size_t(nets[i].id))};
        if (!route_ctx.route_trees[pid]) continue;

        VtrRouteData& route = routes[i];
        for (const RouteTreeNode& node : route_ctx.route_trees[pid]->all_nodes()) {
            route.used_nodes.insert(size_t(node.inode));
            if (node.parent()) {
                route.parent[size_t(node.inode)] = size_t(node.parent()->inode);
                size_t edge_id = find_edge(
                    size_t(node.parent()->inode), size_t(node.inode),
                    size_t(node.parent_switch));
                if (edge_id != std::numeric_limits<size_t>::max()) {
                    route.parent_edge[size_t(node.inode)] = edge_id;
                }
            }
        }

        route.complete = route.used_nodes.count(nets[i].source) != 0;
        for (size_t sink : nets[i].sinks) {
            size_t node = sink;
            size_t steps = 0;
            if (!route.used_nodes.count(sink)) route.complete = false;
            while (route.complete && node != nets[i].source) {
                auto parent_it = route.parent.find(node);
                if (parent_it == route.parent.end()
                    || ++steps > route.used_nodes.size()) {
                    route.complete = false;
                    break;
                }
                size_t parent = parent_it->second;
                if (!route.parent_edge.count(node)
                    || edges[route.parent_edge.at(node)].tail != parent) {
                    route.complete = false;
                    break;
                }
                node = parent;
            }
        }
    }
    return routes;
}

NetDomain make_net_domain(const NetData& net,
                          const VtrRouteData& route,
                          const std::vector<Edge>& edges,
                          size_t num_nodes,
                          bool bbox_enabled,
                          int margin) {
    const bool use_full_graph = !bbox_enabled || !route.complete;
    std::vector<unsigned char> selected(num_nodes, use_full_graph ? 1 : 0);

    if (!use_full_graph) {
        BoundingBox box;
        include_node_in_box(box, net.source);
        for (size_t sink : net.sinks) include_node_in_box(box, sink);
        for (size_t node : route.used_nodes) include_node_in_box(box, node);

        for (size_t node = 0; node < num_nodes; node++) {
            if (node_intersects_box(node, box, margin)) selected[node] = 1;
        }

        // Inclusão explícita protege a rota VTR até contra particularidades de
        // coordenadas de SOURCE/SINK e segmentos longos do RRGraph.
        selected[net.source] = 1;
        for (size_t sink : net.sinks) selected[sink] = 1;
        for (size_t node : route.used_nodes) selected[node] = 1;
    }

    NetDomain domain;
    domain.fallback_full_graph = bbox_enabled && use_full_graph;
    for (size_t node = 0; node < num_nodes; node++) {
        if (selected[node]) domain.nodes.push_back(node);
    }

    for (size_t edge_id = 0; edge_id < edges.size(); edge_id++) {
        const Edge& edge = edges[edge_id];
        if (!selected[edge.tail] || !selected[edge.head]) continue;
        domain.edges.push_back(edge_id);
    }
    return domain;
}

// BFS no RRGraph restrito aos nós selecionados pela net, a partir do source.
// Grava parent_node/parent_edge para permitir o walkback até cada sink.
struct NetBfs {
    std::unordered_set<size_t>           visited;
    std::unordered_map<size_t, size_t>   parent_node;
    std::unordered_map<size_t, RREdgeId> parent_edge;
};

// BFS restrito às arestas que efetivamente carregam fluxo na solução. Isso
// mantém a route_tree exportada alinhada ao segundo critério do objetivo, em
// vez de escolher caminhos arbitrários no subgrafo induzido pelos nós x = 1.
NetBfs bfs_positive_flow(const NetData& net,
                         const NetDomain& domain,
                         const std::vector<Edge>& edges,
                         const IloCplex& cplex,
                         const IloNumVarArray& flow) {
    constexpr double flow_epsilon = 1e-7;

    NetBfs bfs;
    std::queue<size_t> queue;
    queue.push(net.source);
    bfs.visited.insert(net.source);

    std::unordered_map<size_t, std::vector<size_t>> positive_out;
    for (size_t local_e = 0; local_e < domain.edges.size(); local_e++) {
        if (cplex.getValue(flow[local_e]) > flow_epsilon) {
            size_t global_e = domain.edges[local_e];
            positive_out[edges[global_e].tail].push_back(global_e);
        }
    }

    while (!queue.empty()) {
        size_t node = queue.front();
        queue.pop();
        auto out_it = positive_out.find(node);
        if (out_it == positive_out.end()) continue;
        for (size_t global_e : out_it->second) {
            size_t sink = edges[global_e].head;
            if (bfs.visited.count(sink)) continue;

            bfs.visited.insert(sink);
            bfs.parent_node[sink] = node;
            bfs.parent_edge[sink] =
                RREdgeId(edges[global_e].rr_edge_id);
            queue.push(sink);
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

// Soma os proxies RC das arestas de um caminho source->sink.
double sum_path_proxy_ns(const routing::PathResult& path,
                         const std::vector<double>& proxy_by_rr_edge) {
    double sum = 0.0;
    for (RREdgeId e : path.edges) {
        size_t idx = static_cast<size_t>(e);
        if (idx < proxy_by_rr_edge.size()) sum += proxy_by_rr_edge[idx];
    }
    return sum;
}

// Compara os delays Elmore (source→sink) dos dois roteamentos.
void print_delay_comparison(std::ostream& out,
                            const std::filesystem::path& csv_path,
                            const std::vector<NetData>& nets,
                            const NetPinsMatrix<float>& vtr_delay,
                            const NetPinsMatrix<float>& ilp_delay,
                            const std::vector<std::vector<double>>& proxy_per_sink) {
    out << "\n========== DELAY VTR x ILP ==========\n";
    out << std::left << std::setw(24) << "Net"
        << std::setw(16) << "VTR max (ns)"
        << std::setw(16) << "ILP max (ns)"
        << "ILP/VTR\n";

    float vtr_max = 0, ilp_max = 0;
    double vtr_sum = 0, ilp_sum = 0;
    size_t n_sinks = 0;

    for (const NetData& net : nets) {
        ParentNetId pid{static_cast<int>(size_t(net.id))};
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

    out << "\n========== PROXY vs ELMORE REAL (ILP) ==========\n";
    out << std::left << std::setw(24) << "Net"
        << std::setw(8) << "Sink"
        << std::setw(14) << "proxy(ns)"
        << std::setw(14) << "elmore(ns)"
        << std::setw(14) << "abs(ns)"
        << std::setw(12) << "rel(%)"
        << "ratio\n";

    std::ofstream csv(csv_path, std::ios::trunc);
    if (csv.is_open()) {
        csv << "net,sink_idx,proxy_ns,elmore_ns,abs_err_ns,rel_err_pct,ratio\n";
    }

    double sum_abs = 0.0, sum_rel_frac = 0.0;
    double sum_x = 0.0, sum_y = 0.0, sum_x2 = 0.0, sum_y2 = 0.0, sum_xy = 0.0;
    size_t n = 0;

    out << std::fixed << std::setprecision(4);
    for (size_t i = 0; i < nets.size(); i++) {
        const NetData& net = nets[i];
        ParentNetId pid{static_cast<int>(size_t(net.id))};
        for (size_t j = 0; j < net.sinks.size(); j++) {
            double proxy_ns = proxy_per_sink[i][j];
            double elmore_ns = double(ilp_delay[pid][j + 1]) * 1e9;
            double abs_err = std::fabs(proxy_ns - elmore_ns);
            double rel_pct = (elmore_ns > 0.0)
                                 ? (abs_err / elmore_ns * 100.0)
                                 : std::numeric_limits<double>::infinity();
            double ratio = (elmore_ns > 0.0)
                               ? (proxy_ns / elmore_ns)
                               : std::numeric_limits<double>::infinity();

            out << std::left << std::setw(24) << net.name
                << std::setw(8) << j
                << std::setw(14) << proxy_ns
                << std::setw(14) << elmore_ns
                << std::setw(14) << abs_err
                << std::setw(12) << rel_pct
                << ratio << "\n";

            if (csv.is_open()) {
                csv << net.name << "," << j << "," << proxy_ns << ","
                    << elmore_ns << "," << abs_err << "," << rel_pct << ","
                    << ratio << "\n";
            }

            sum_abs += abs_err;
            if (elmore_ns > 0.0) sum_rel_frac += abs_err / elmore_ns;
            sum_x += proxy_ns;
            sum_y += elmore_ns;
            sum_x2 += proxy_ns * proxy_ns;
            sum_y2 += elmore_ns * elmore_ns;
            sum_xy += proxy_ns * elmore_ns;
            n++;
        }
    }

    out << "-------------------------------------\n";
    out << "Sinks analisados: " << n << "\n";
    if (n > 0) {
        out << "MAE  (ns)       : " << (sum_abs / n) << "\n";
        out << "MAPE (%)        : " << (sum_rel_frac / n * 100.0) << "\n";
        double num = double(n) * sum_xy - sum_x * sum_y;
        double den2 = (double(n) * sum_x2 - sum_x * sum_x) *
                      (double(n) * sum_y2 - sum_y * sum_y);
        if (den2 > 0.0) {
            out << "Correlacao Pearson: " << (num / std::sqrt(den2)) << "\n";
        } else {
            out << "Correlacao Pearson: n/a\n";
        }
    }
    out << "=====================================\n";
    if (csv.is_open()) {
        out << "[proxy] CSV salvo em: " << csv_path << "\n";
    } else {
        out << "[proxy] aviso: nao foi possivel salvar CSV em " << csv_path
            << "\n";
    }
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
                           const std::vector<NetDomain>& domains,
                           const std::vector<NetBfs>&   net_bfs,
                           const std::vector<std::vector<size_t>>& preds) {
    const auto& rr_graph = g_vpr_ctx.device().rr_graph;
    const size_t k = nets.size();

    out << "\n========== DIAGNÓSTICO ILP_DIAG ==========\n";

    size_t total_disc = 0, total_orphans = 0, orphans_zero_cost = 0;

    for (size_t i = 0; i < k; i++) {
        size_t reached = count_reachable_sinks(nets[i], net_bfs[i]);
        if (reached == nets[i].sinks.size()) continue;
        total_disc++;

        std::unordered_set<size_t> used;
        for (size_t local = 0; local < domains[i].nodes.size(); local++) {
            if (cplex.getValue(x[i][local]) > 0.5)
                used.insert(domains[i].nodes[local]);
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
    std::filesystem::path csv_path = out_dir / "proxy_vs_elmore.csv";
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
    out << "[elmore] computed via VTR load_net_delay_from_routing()\n";

    std::vector<NetData> nets = collect_nets();
    const size_t k = nets.size();

    // Lista de predecessores: preds[v] = {u : (u,v) ∈ E}
    std::vector<std::vector<size_t>> preds(num_nodes);
    for (RRNodeId node : rr_graph.nodes()) {
        for (t_edge_size i = 0; i < rr_graph.num_edges(node); i++) {
            preds[size_t(rr_graph.edge_sink_node(node, i))].push_back(size_t(node));
        }
    }

    // Índice global de arestas dirigidas, para as variáveis de fluxo.
    // O proxy RC aproxima o incremento de Elmore local da aresta. Ele é
    // convertido para ns para manter o segundo objetivo bem escalado.
    out << "[proxy] precomputing edge delays "
           "(Tdel + R*Chead + 0.5*Rhead*Chead) ...\n";
    std::vector<Edge>                edges;
    std::vector<std::vector<size_t>> out_edges(num_nodes);
    for (RRNodeId node : rr_graph.nodes()) {
        size_t u = size_t(node);
        RREdgeId first_edge = rr_graph.node_first_edge(node);
        for (t_edge_size i = 0; i < rr_graph.num_edges(node); i++) {
            RRNodeId head = rr_graph.edge_sink_node(node, i);
            size_t w = size_t(head);
            RRSwitchId switch_id(rr_graph.edge_switch(node, i));
            const t_rr_switch_inf& rr_switch =
                rr_graph.rr_switch_inf(switch_id);
            double head_capacitance =
                double(rr_graph.node_C(head)) + double(rr_switch.Cinternal);
            double delay_proxy_s =
                double(rr_switch.Tdel)
                + double(rr_switch.R) * head_capacitance
                + 0.5 * double(rr_graph.node_R(head))
                          * double(rr_graph.node_C(head));
            size_t e = edges.size();
            edges.push_back({
                u, w, size_t(first_edge) + i, size_t(switch_id),
                delay_proxy_s * 1e9});
            out_edges[u].push_back(e);
        }
    }
    const size_t num_edges = edges.size();
    std::vector<double> proxy_by_rr_edge(num_edges, 0.0);
    for (const Edge& e : edges) {
        proxy_by_rr_edge[e.rr_edge_id] = e.delay_proxy_ns;
    }

    const bool bbox_enabled = env_enabled("ILP_BBOX", true);
    const int bbox_margin = bbox_margin_from_env(out);
    std::vector<VtrRouteData> vtr_routes = collect_vtr_routes(nets, edges, out_edges);
    std::vector<NetDomain> domains;
    domains.reserve(k);

    size_t candidate_nodes = 0;
    size_t candidate_edges = 0;
    size_t fallback_nets = 0;
    for (size_t i = 0; i < k; i++) {
        domains.push_back(make_net_domain(nets[i], vtr_routes[i], edges,
                                          num_nodes, bbox_enabled, bbox_margin));
        candidate_nodes += domains.back().nodes.size();
        candidate_edges += domains.back().edges.size();
        if (domains.back().fallback_full_graph) fallback_nets++;
    }

    const size_t dense_nodes = k * num_nodes;
    const size_t dense_edges = k * num_edges;
    auto reduction = [](size_t kept, size_t dense) {
        return dense == 0 ? 0.0 : 100.0 * (1.0 - double(kept) / double(dense));
    };

    out << "\n>>> Bounding box: " << (bbox_enabled ? "ON" : "OFF")
        << (bbox_enabled ? " (margem " + std::to_string(bbox_margin) + " tiles)" : "")
        << "\n";
    if (fallback_nets > 0) {
        out << ">>> Bounding box: " << fallback_nets
            << " nets sem warm start verificável mantidas no grafo completo\n";
    }
    out << ">>> ILP: " << k << " nets, " << num_nodes << " nós RR, "
        << num_edges << " arestas RR globais\n";
    out << ">>> Domínio x: " << candidate_nodes << " / " << dense_nodes
        << " variáveis (redução " << reduction(candidate_nodes, dense_nodes)
        << " %)\n";
    out << ">>> Domínio f: " << candidate_edges << " / " << dense_edges
        << " variáveis (redução " << reduction(candidate_edges, dense_edges)
        << " %)\n";

    if (env_enabled("ILP_BBOX_VERBOSE", false)) {
        for (size_t i = 0; i < k; i++) {
            out << "    Net " << nets[i].name << ": "
                << domains[i].nodes.size() << "/" << num_nodes << " nós, "
                << domains[i].edges.size() << "/" << num_edges << " arestas"
                << (domains[i].fallback_full_graph ? " [fallback completo]" : "")
                << "\n";
        }
    }

    // Tempo de build do modelo (construção + warm start), medido separado do
    // solve para a tabela de escalabilidade — o comparativo científico usa só o
    // tempo de solve, mas o tempo de build faz parte da história de escala.
    auto t_build0 = std::chrono::steady_clock::now();

    IloEnv env;
    try {
        IloModel model(env);

        // x[i][local_v] ∈ {0,1} — nó global domains[i].nodes[local_v]
        // usado pela net i. Nós ausentes do domínio estão fixados em zero.
        std::vector<IloBoolVarArray> x;
        x.reserve(k);
        for (size_t i = 0; i < k; i++) {
            x.emplace_back(env, (IloInt)domains[i].nodes.size());
        }

        // f[i][local_e] ∈ [0, D_i] — fluxo single-commodity na aresta global
        // domains[i].edges[local_e]. D_i = nº de sinks.
        std::vector<IloNumVarArray> f;
        f.reserve(k);
        for (size_t i = 0; i < k; i++) {
            IloNum cap_flow = (IloNum)nets[i].sinks.size();
            f.emplace_back(env, (IloInt)domains[i].edges.size(), 0.0, cap_flow);
        }

        // Objetivo lexicográfico:
        //   1) preserva o objetivo original de congestionamento (base_cost);
        //   2) entre soluções de mesmo custo, minimiza o proxy RC agregado dos
        //      caminhos source→sink. Como f conta a demanda a jusante, uma
        //      aresta compartilhada é ponderada pelo número de sinks atendidos.
        IloExpr base_cost_obj(env);
        for (size_t i = 0; i < k; i++) {
            for (size_t local = 0; local < domains[i].nodes.size(); local++) {
                size_t node = domains[i].nodes[local];
                base_cost_obj +=
                    (IloNum)get_single_rr_cong_base_cost(RRNodeId(node))
                    * x[i][local];
            }
        }
        IloExpr delay_proxy_obj(env);
        for (size_t i = 0; i < k; i++) {
            for (size_t local_e = 0; local_e < domains[i].edges.size(); local_e++) {
                const Edge& edge = edges[domains[i].edges[local_e]];
                delay_proxy_obj += edge.delay_proxy_ns * f[i][local_e];
            }
        }
        model.add(IloMinimize(
            env, IloStaticLex(env, base_cost_obj, delay_proxy_obj)));

        // 1. Capacidade de vértice: Σ_i x[i][v] ≤ cap(v)
        //    cap(v) vem do RRGraph: SOURCE/SINK podem ter capacidade > 1;
        //    fixar 1 nesses nós tornaria o modelo infeasible sem necessidade.
        for (size_t v = 0; v < num_nodes; v++) {
            IloExpr sum(env);
            for (size_t i = 0; i < k; i++) {
                size_t local = local_index(domains[i].nodes, v);
                if (local != std::numeric_limits<size_t>::max()) sum += x[i][local];
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

            // Adjacência local temporária: existe somente enquanto as
            // restrições desta net são criadas, evitando manter milhões de
            // pequenos vetores vivos ao mesmo tempo nos circuitos grandes.
            std::vector<size_t> node_local(
                num_nodes, std::numeric_limits<size_t>::max());
            for (size_t local = 0; local < domains[i].nodes.size(); local++) {
                node_local[domains[i].nodes[local]] = local;
            }
            std::vector<std::vector<size_t>> local_out(domains[i].nodes.size());
            std::vector<std::vector<size_t>> local_in(domains[i].nodes.size());
            for (size_t local_e = 0; local_e < domains[i].edges.size(); local_e++) {
                const Edge& edge = edges[domains[i].edges[local_e]];
                local_out[node_local[edge.tail]].push_back(local_e);
                local_in[node_local[edge.head]].push_back(local_e);
            }

            // 2a. Conservação: (saída) − (entrada) = oferta − demanda em cada nó.
            for (size_t local_v = 0; local_v < domains[i].nodes.size(); local_v++) {
                size_t v = domains[i].nodes[local_v];
                IloExpr balance(env);
                for (size_t local_e : local_out[local_v])
                    balance += f[i][local_e];
                for (size_t local_e : local_in[local_v])
                    balance -= f[i][local_e];

                IloNum rhs = 0.0;
                if (v == nets[i].source)   rhs = D;       // source emite D_i
                else if (sink_set.count(v)) rhs = -1.0;    // cada sink consome 1
                model.add(balance == rhs);
                balance.end();
            }

            // 2b. Acoplamento fluxo↔nó: aresta só conduz fluxo se ambos os
            //     extremos estão usados (x = 1).
            for (size_t local_e = 0; local_e < domains[i].edges.size(); local_e++) {
                const Edge& edge = edges[domains[i].edges[local_e]];
                size_t tail = node_local[edge.tail];
                size_t head = node_local[edge.head];
                model.add(f[i][local_e] <= D * x[i][tail]);
                model.add(f[i][local_e] <= D * x[i][head]);
            }
        }

        // 3. Source e sinks obrigatórios
        for (size_t i = 0; i < k; i++) {
            size_t source = local_index(domains[i].nodes, nets[i].source);
            model.add(x[i][source] == 1);
            for (size_t t : nets[i].sinks) {
                size_t sink = local_index(domains[i].nodes, t);
                model.add(x[i][sink] == 1);
            }
        }

        IloCplex cplex(model);
        cplex.setParam(IloCplex::Param::TimeLimit, cfg.time_limit);

        // Warm start: a solução do VTR (route_trees) como MIPStart completo.
        // Setamos tanto x quanto f — com fluxo, um MIPStart só com x seria
        // incompleto e o CPLEX poderia descartá-lo. O fluxo da árvore VTR é
        // reconstruído contando, por aresta, quantos sinks há na subárvore abaixo
        // dela (= soma das demandas que passam pela aresta no sentido source→sink).
        double warm_start_base_cost = 0.0;
        double warm_start_delay_proxy_ns = 0.0;
        bool has_complete_warm_start = false;
        {
            IloNumVarArray start_vars(env);
            IloNumArray    start_vals(env);
            size_t nets_with_start = 0;

            for (size_t i = 0; i < k; i++) {
                if (!vtr_routes[i].complete) continue;
                nets_with_start++;
                const VtrRouteData& route = vtr_routes[i];

                for (size_t node : route.used_nodes) {
                    warm_start_base_cost +=
                        get_single_rr_cong_base_cost(RRNodeId(node));
                    if (local_index(domains[i].nodes, node)
                        == std::numeric_limits<size_t>::max()) {
                        throw std::runtime_error(
                            "bounding box não preservou um nó da rota VTR");
                    }
                }

                // Fluxo por aresta: +1 em cada aresta do caminho source→sink, por sink.
                std::vector<double> flow(domains[i].edges.size(), 0.0);
                for (size_t t : nets[i].sinks) {
                    size_t node = t;
                    while (node != nets[i].source) {
                        size_t par = route.parent.at(node);
                        size_t global_e = route.parent_edge.at(node);
                        size_t local_e = local_index(domains[i].edges,
                                                     global_e);
                        if (local_e == std::numeric_limits<size_t>::max()) {
                            throw std::runtime_error(
                                "bounding box não preservou uma aresta da rota VTR");
                        }
                        flow[local_e] += 1.0;
                        warm_start_delay_proxy_ns +=
                            edges[global_e].delay_proxy_ns;
                        node = par;
                    }
                }

                for (size_t local = 0; local < domains[i].nodes.size(); local++) {
                    start_vars.add(x[i][local]);
                    start_vals.add(route.used_nodes.count(domains[i].nodes[local])
                                       ? 1.0 : 0.0);
                }
                for (size_t local = 0; local < domains[i].edges.size(); local++) {
                    start_vars.add(f[i][local]);
                    start_vals.add(flow[local]);
                }
            }

            if (nets_with_start == k) {
                cplex.addMIPStart(start_vars, start_vals);
                has_complete_warm_start = true;
                out << ">>> Warm start: solução VTR fornecida como MIPStart ("
                    << nets_with_start << "/" << k << " nets, base_cost "
                    << warm_start_base_cost << ", proxy RC "
                    << warm_start_delay_proxy_ns << " ns)\n";
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

        double base_cost_value = cplex.getValue(base_cost_obj);
        double delay_proxy_value_ns = cplex.getValue(delay_proxy_obj);
        IloInt multiobj_solves = cplex.getMultiObjNsolves();
        out << "Objetivo primário (base_cost)    : " << base_cost_value << "\n";
        out << "Objetivo secundário (proxy RC)   : "
            << delay_proxy_value_ns << " ns\n";
        out << "Subproblemas multiobjetivo       : "
            << multiobj_solves << "\n";
        if (multiobj_solves > 0) {
            double primary_bound = cplex.getMultiObjInfo(
                IloCplex::MultiObjBestObjValue, 0);
            double primary_gap =
                std::abs(base_cost_value - primary_bound)
                / std::max(1e-10, std::abs(base_cost_value));
            out << "Melhor bound (base_cost)         : " << primary_bound << "\n";
            out << "Gap relativo (base_cost)         : "
                << primary_gap * 100.0 << " %\n";
        }
        out << "Nós B&B explorados               : " << cplex.getNnodes() << "\n";
        out << "Variáveis (x / f)                : "
            << candidate_nodes << " / " << candidate_edges << "\n";
        out << "Variáveis densas evitadas (x / f): "
            << (dense_nodes - candidate_nodes) << " / "
            << (dense_edges - candidate_edges) << "\n";
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

        // Proteção de qualidade: como toda a rota VTR está no subgrafo, a
        // solução minimizada nunca deve ser pior que o próprio warm start no
        // objetivo da formulação. Se isso ocorrer, preservamos as route_trees
        // originais do VTR e não exportamos a solução suspeita.
        if (has_complete_warm_start
            && base_cost_value > warm_start_base_cost + 1e-6) {
            out << "ERRO: base_cost ILP (" << base_cost_value
                << ") pior que o warm start VTR (" << warm_start_base_cost
                << "). Rota VTR preservada; saída ILP descartada.\n";
            env.end();
            return;
        }

        // Extrair nós usados e verificar conectividade nas arestas que
        // efetivamente carregam fluxo.
        std::vector<NetBfs> net_bfs(k);
        size_t nets_ok = 0;
        for (size_t i = 0; i < k; i++) {
            std::unordered_set<size_t> used;
            for (size_t local = 0; local < domains[i].nodes.size(); local++) {
                if (cplex.getValue(x[i][local]) > 0.5)
                    used.insert(domains[i].nodes[local]);
            }
            net_bfs[i] = bfs_positive_flow(
                nets[i], domains[i], edges, cplex, f[i]);
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
            diagnose_disconnected(out, nets, cplex, x, domains, net_bfs, preds);
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
        std::vector<std::vector<double>> proxy_per_sink(k);
        for (size_t i = 0; i < k; i++) {
            ParentNetId pid{static_cast<int>(size_t(nets[i].id))};
            if (route_ctx.route_trees[pid]) {
                pathfinder_update_cost_from_route_tree(
                    route_ctx.route_trees[pid]->root(), -1);
                route_ctx.route_trees[pid] = vtr::nullopt;
            }
            proxy_per_sink[i].resize(nets[i].sinks.size());
            for (size_t j = 0; j < nets[i].sinks.size(); j++) {
                routing::PathResult path =
                    walkback_path(net_bfs[i], nets[i].source, nets[i].sinks[j]);
                proxy_per_sink[i][j] = sum_path_proxy_ns(path, proxy_by_rr_edge);
                routing::mirror_path_to_vtr(pid, /*sink_pin_index=*/(int)(j + 1), path);
            }
        }

        NetPinsMatrix<float> ilp_delay = make_net_pins_matrix<float>((const Netlist<>&)nlist);
        load_net_delay_from_routing((const Netlist<>&)nlist, ilp_delay);
        out << "[elmore] computed via VTR load_net_delay_from_routing()\n";

        print_delay_comparison(out, csv_path, nets, vtr_delay, ilp_delay,
                               proxy_per_sink);
    } catch (IloException& e) {
        std::cerr << "Erro CPLEX: " << e.getMessage() << "\n";
    } catch (const std::exception& e) {
        std::cerr << "Erro ao construir o domínio ILP: " << e.what() << "\n";
    }
    env.end();
}
