#include "routing/steiner_router.h"
#include "vpr_context.h"
#include "vpr_api.h"
#include <iostream>
#include <limits>
#include <cmath>

namespace routing {

void RoutingResult::print_summary() const {
    std::cout << "\n=== Routing Summary ===\n";
    std::cout << "Total nets: " << total_nets << "\n";
    std::cout << "Nets routed: " << nets_routed << "\n";
    if (total_nets > 0) {
        std::cout << "Success rate: " << (100.0f * nets_routed / total_nets) << "%\n";
    }
    std::cout << "Congestion violations: " << congestion_violations << "\n";
}

SteinerRouter::SteinerRouter(float congestion_weight)
    : congestion_weight_(congestion_weight) {}

void SteinerRouter::build_rr_node_index(const RRGraphView& rr_graph) {
    rr_node_index_.clear();

    for (RRNodeId node_id : rr_graph.nodes()) {
        e_rr_type type = rr_graph.node_type(node_id);

        // Indexar apenas OPIN e IPIN — os únicos que correspondem a pins físicos
        if (type != OPIN && type != IPIN) continue;

        int x   = rr_graph.node_xlow(node_id);
        int y   = rr_graph.node_ylow(node_id);
        int ptc = rr_graph.node_ptc_num(node_id);

        rr_node_index_[{x, y, ptc, type}] = node_id;
    }

    std::cout << "RR node index built: " << rr_node_index_.size() << " OPIN/IPIN nodes indexed.\n";
}

RRNodeId SteinerRouter::find_rr_node_for_pin(
    ClusterPinId pin_id,
    const ClusteredNetlist& netlist,
    const RRGraphView& rr_graph,
    e_rr_type expected_type) const {

    // Posição do bloco no chip
    ClusterBlockId block_id = netlist.pin_block(pin_id);
    auto& place_ctx = g_vpr_ctx.placement();
    int x = place_ctx.block_locs()[block_id].loc.x;
    int y = place_ctx.block_locs()[block_id].loc.y;

    // Índice lógico do pin dentro do bloco
    int pin_index = netlist.pin_logical_index(pin_id);

    // Lookup direto no índice — O(log n) em vez de varredura linear
    auto it = rr_node_index_.find({x, y, pin_index, expected_type});
    if (it != rr_node_index_.end()) {
        return it->second;
    }

    return RRNodeId::INVALID();
}

Point SteinerRouter::get_block_location(
    ClusterBlockId block_id,
    const ClusteredNetlist& netlist) const {
    auto& place_ctx = g_vpr_ctx.placement();
    const auto& block_loc = place_ctx.block_locs()[block_id];

    return Point(static_cast<float>(block_loc.loc.x), static_cast<float>(block_loc.loc.y));
}

RRNodeId SteinerRouter::find_nearest_rr_node(
    const Point& location,
    const RRGraphView& rr_graph,
    e_rr_type preferred_type) const {

    RRNodeId best_node;
    float best_distance = std::numeric_limits<float>::max();

    for (RRNodeId node_id : rr_graph.nodes()) {
        if (rr_graph.node_type(node_id) != preferred_type) continue;

        float nx = static_cast<float>(rr_graph.node_xlow(node_id));
        float ny = static_cast<float>(rr_graph.node_ylow(node_id));
        float distance = std::abs(location.x - nx) + std::abs(location.y - ny);

        if (distance < best_distance) {
            best_distance = distance;
            best_node = node_id;
        }
    }

    return best_node;
}

NetRoutingResult SteinerRouter::route_net(
    ClusterNetId net_id,
    const ClusteredNetlist& netlist,
    const RRGraphView& rr_graph) {

    NetRoutingResult result;
    result.success = false;
    result.sinks_total = netlist.net_sinks(net_id).size();
    result.sinks_routed = 0;

    // Obter driver pin e mapear para o nó RR correto (OPIN)
    ClusterPinId driver_pin = netlist.net_driver(net_id);
    if (!driver_pin.is_valid()) {
        return result;
    }

    RRNodeId source_node = find_rr_node_for_pin(driver_pin, netlist, rr_graph, OPIN);
    if (!source_node.is_valid()) {
        std::cerr << "Aviso: nó RR não encontrado para driver da net " << size_t(net_id) << "\n";
        return result;
    }

    ClusterBlockId driver_block = netlist.pin_block(driver_pin);
    Point source_location = get_block_location(driver_block, netlist);

    // Coletar sinks — mapeando cada um para seu nó RR correto (IPIN)
    std::vector<Point> sink_locations;
    std::vector<RRNodeId> sink_nodes;

    for (ClusterPinId sink_pin : netlist.net_sinks(net_id)) {
        RRNodeId sink_node = find_rr_node_for_pin(sink_pin, netlist, rr_graph, IPIN);
        if (!sink_node.is_valid()) {
            std::cerr << "Aviso: nó RR não encontrado para sink da net " << size_t(net_id) << "\n";
            continue;
        }

        ClusterBlockId sink_block = netlist.pin_block(sink_pin);
        sink_locations.push_back(get_block_location(sink_block, netlist));
        sink_nodes.push_back(sink_node);
    }

    if (sink_nodes.empty()) {
        return result;
    }

    // Encontrar ponto de Steiner (hub geométrico central)
    Point steiner_point = find_steiner_point(source_location, sink_locations, 1.0f);

    // Encontrar nó CHANX mais próximo do ponto de Steiner
    RRNodeId steiner_node = find_nearest_rr_node(steiner_point, rr_graph, CHANX);
    if (!steiner_node.is_valid()) {
        steiner_node = source_node;  // Fallback: partir direto da source
    }

    // Rotear source -> cada sink via PathFinder (Dijkstra com congestionamento)
    PathFinder path_finder(rr_graph, congestion_map_);

    for (size_t i = 0; i < sink_nodes.size(); ++i) {
        PathFindingParams params;
        params.source = source_node;
        params.sink = sink_nodes[i];
        params.congestion_weight = congestion_weight_;

        PathResult path = path_finder.find_path(params);

        if (path.success) {
            result.sink_paths.push_back(path.edges);
            result.sinks_routed++;
            path_finder.commit_path(path);
        }
    }

    result.success = (result.sinks_routed == result.sinks_total);
    return result;
}

RoutingResult SteinerRouter::route(
    const ClusteredNetlist& netlist,
    const RRGraphView& rr_graph) {

    RoutingResult result;
    result.total_nets = netlist.nets().size();
    result.nets_routed = 0;
    result.congestion_violations = 0;

    // Construir índice espacial uma única vez antes de rotear
    build_rr_node_index(rr_graph);

    std::cout << "Starting routing of " << result.total_nets << " nets...\n";

    for (ClusterNetId net_id : netlist.nets()) {
        NetRoutingResult net_result = route_net(net_id, netlist, rr_graph);

        if (net_result.success) {
            result.nets_routed++;
        }

        result.net_results.push_back(net_result);

        if ((result.net_results.size() % 100) == 0) {
            std::cout << "Routed " << result.net_results.size() << "/" << result.total_nets << " nets\n";
        }
    }

    return result;
}

} // namespace routing