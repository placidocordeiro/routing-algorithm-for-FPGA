#include "routing/steiner_router.h"
#include "vpr_context.h"
#include "vpr_api.h"
#include "route_tree.h"
#include "route_common.h"
#include <iostream>
#include <limits>
#include <cmath>

namespace routing
{
    void RoutingResult::print_summary() const
    {
        std::cout << "\n=== Routing Summary ===\n";
        std::cout << "Total nets: " << total_nets << "\n";
        std::cout << "Nets routed: " << nets_routed << "\n";
        if (total_nets > 0)
        {
            std::cout << "Success rate: " << (100.0f * nets_routed / total_nets) << "%\n";
        }
        std::cout << "Congestion violations: " << congestion_violations << "\n";
    }

    SteinerRouter::SteinerRouter(float congestion_weight)
        : congestion_weight_(congestion_weight) {}

    RRNodeId SteinerRouter::find_nearest_rr_node(
        const Point &location,
        const RRGraphView &rr_graph,
        e_rr_type preferred_type) const
    {

        RRNodeId best_node;
        float best_distance = std::numeric_limits<float>::max();

        for (RRNodeId node_id : rr_graph.nodes())
        {
            if (rr_graph.node_type(node_id) != preferred_type)
                continue;

            float nx = static_cast<float>(rr_graph.node_xlow(node_id));
            float ny = static_cast<float>(rr_graph.node_ylow(node_id));
            float distance = std::abs(location.x - nx) + std::abs(location.y - ny);

            if (distance < best_distance)
            {
                best_distance = distance;
                best_node = node_id;
            }
        }

        return best_node;
    }

    NetRoutingResult SteinerRouter::route_net(
        ClusterNetId net_id,
        const ClusteredNetlist &netlist,
        const RRGraphView &rr_graph)
    {

        NetRoutingResult result;
        result.success = false;
        result.sinks_total = netlist.net_sinks(net_id).size();
        result.sinks_routed = 0;

        auto &route_ctx = g_vpr_ctx.mutable_routing();
        const auto &net_terminals = route_ctx.net_rr_terminals[net_id];

        // net_terminals[0] = SOURCE (driver), net_terminals[1..N] = SINKs
        if (net_terminals.empty() || !net_terminals[0].is_valid())
        {
            return result;
        }

        RRNodeId source_node = net_terminals[0];

        Point source_location(
            static_cast<float>(rr_graph.node_xlow(source_node)),
            static_cast<float>(rr_graph.node_ylow(source_node)));

        // Coletar SINK nodes e suas localizações para o cálculo do ponto de Steiner
        std::vector<Point> sink_locations;
        std::vector<RRNodeId> sink_nodes;

        for (size_t i = 1; i < net_terminals.size(); i++)
        {
            RRNodeId sink_node = net_terminals[i];
            if (!sink_node.is_valid())
                continue;

            sink_locations.push_back(Point(
                static_cast<float>(rr_graph.node_xlow(sink_node)),
                static_cast<float>(rr_graph.node_ylow(sink_node))));
            sink_nodes.push_back(sink_node);
        }

        if (sink_nodes.empty())
        {
            return result;
        }

        // Encontrar ponto de Steiner (hub geométrico central)
        Point steiner_point = find_steiner_point(source_location, sink_locations, 1.0f);

        // Encontrar nó CHANX mais próximo do ponto de Steiner
        RRNodeId steiner_node = find_nearest_rr_node(steiner_point, rr_graph, CHANX);
        if (!steiner_node.is_valid())
        {
            steiner_node = source_node; // Fallback: partir direto da source
        }
        (void)steiner_node; // Usado para guiar futuras otimizações de custo

        // Criar RouteTree para esta net no contexto VTR (raiz = SOURCE)
        route_ctx.route_trees[net_id] = RouteTree(net_id);
        RouteTree &tree = route_ctx.route_trees[net_id].value();

        // Rotear SOURCE -> cada SINK via PathFinder (Dijkstra com congestionamento)
        PathFinder path_finder(rr_graph, congestion_map_);

        // Nós usados pelos caminhos desta net (podem ser reutilizados entre sinks da mesma net)
        std::unordered_set<RRNodeId> current_net_nodes;
        current_net_nodes.insert(source_node); // source sempre pertence à net atual

        for (size_t i = 0; i < sink_nodes.size(); ++i)
        {
            int pin_index = static_cast<int>(i) + 1; // 1-indexed, conforme convenção VTR

            PathFindingParams params;
            params.source = source_node;
            params.sink = sink_nodes[i];
            params.congestion_weight = congestion_weight_;
            // Bloquear nós já comprometidos por nets anteriores.
            // Nós da própria net atual (current_net_nodes) não são bloqueados,
            // pois compartilhá-los é o que constrói a árvore.
            params.blocked_nodes = &occupied_nodes_;

            PathResult path = path_finder.find_path(params);

            if (path.success && path.nodes.size() >= 2)
            {
                // Registrar prev_edge para cada nó no caminho:
                // rr_node_route_inf[nodes[j]].prev_edge = edge que chegou a nodes[j]
                // Isso permite que update_from_heap reconstrua o caminho do sink até a raiz
                for (size_t j = 1; j < path.nodes.size(); j++)
                {
                    route_ctx.rr_node_route_inf[path.nodes[j]].prev_edge = path.edges[j - 1];
                }

                // Adicionar o caminho à RouteTree via interface nativa do VTR
                RTExploredNode heap_node;
                heap_node.index = path.nodes.back();     // nó SINK
                heap_node.prev_edge = path.edges.back(); // aresta de entrada no SINK

                tree.update_from_heap(&heap_node, pin_index, nullptr, /*is_flat=*/false);

                // Acumular nós desta net para bloquear nets futuras
                for (RRNodeId n : path.nodes)
                    current_net_nodes.insert(n);

                result.sink_paths.push_back(path);
                result.sinks_routed++;

                // Marcar arestas como usadas no mapa de congestionamento
                path_finder.commit_path(path);
            }
        }

        // Comprometer os nós desta net no conjunto global:
        // nets futuras não poderão usar esses nós como intermediários
        for (RRNodeId n : current_net_nodes)
            occupied_nodes_.insert(n);

        // Atualizar ocupação global de todos os nós desta net de uma vez
        // (necessário para feasible_routing() e métricas de overuse)
        if (result.sinks_routed > 0)
        {
            pathfinder_update_cost_from_route_tree(tree.root(), +1);
        }

        result.success = (result.sinks_routed == result.sinks_total);
        return result;
    }

    RoutingResult SteinerRouter::route(
        const ClusteredNetlist &netlist,
        const RRGraphView &rr_graph)
    {

        RoutingResult result;
        result.total_nets = netlist.nets().size();
        result.nets_routed = 0;
        result.congestion_violations = 0;

        // Limpar estado entre chamadas consecutivas (e.g., rodadas W_min vs W_130)
        occupied_nodes_.clear();
        congestion_map_.clear();

        // Inicializar estruturas de roteamento do VTR antes de rotear
        // alloc_and_load: aloca rr_node_route_inf (ocupação, prev_edge, custos por nó)
        // init_route_structs: popula net_rr_terminals (SOURCE/SINK por net), route_bb, etc.
        alloc_and_load_rr_node_route_structs();
        // O VPR usa cast explícito para converter ClusteredNetlist -> Netlist<> (ver vpr_api.cpp:426)
        init_route_structs((const Netlist<> &)netlist, /*bb_factor=*/3, /*has_choking_point=*/false, /*is_flat=*/false);

        std::cout << "Starting routing of " << result.total_nets << " nets...\n";

        for (ClusterNetId net_id : netlist.nets())
        {
            NetRoutingResult net_result = route_net(net_id, netlist, rr_graph);

            if (net_result.success)
            {
                result.nets_routed++;
            }

            result.net_results.push_back(net_result);

            if ((result.net_results.size() % 100) == 0)
            {
                std::cout << "Routed " << result.net_results.size() << "/" << result.total_nets << " nets\n";
            }
        }

        return result;
    }
} // namespace routing