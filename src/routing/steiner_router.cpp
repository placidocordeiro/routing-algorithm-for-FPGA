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
    t_rr_type preferred_type) const {

    RRNodeId best_node;
    float best_distance = std::numeric_limits<float>::max();

    // Iteração simplificada sobre nós (idealmente teria indexação espacial)
    for (RRNodeId node_id : rr_graph.nodes()) {
        if (rr_graph.node_type(node_id) != preferred_type) {
            continue;
        }

        // Placeholder: usar heurística baseada em tipo
        // Em produção, seria necessário acessar coordenadas espaciais do nó
        float distance = 0.0f;

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

    // Obter driver pin
    ClusterPinId driver_pin = netlist.net_driver(net_id);
    if (!driver_pin.is_valid()) {
        return result;
    }

    // TODO: Obter nó RR correspondente ao pin driver
    // Por enquanto, usar primeiro nó da netlist como proxy
    RRNodeId source_node;
    if (rr_graph.nodes().size() > 0) {
        auto nodes = rr_graph.nodes();
        if (!nodes.empty()) {
            source_node = *nodes.begin();
        }
    }

    if (!source_node.is_valid()) {
        return result;
    }

    ClusterBlockId driver_block = netlist.pin_block(driver_pin);
    Point source_location = get_block_location(driver_block, netlist);

    // Coletar sinks
    std::vector<Point> sink_locations;
    std::vector<ClusterPinId> sink_pins;
    std::vector<RRNodeId> sink_nodes;

    for (ClusterPinId sink_pin : netlist.net_sinks(net_id)) {
        ClusterBlockId sink_block = netlist.pin_block(sink_pin);
        Point sink_loc = get_block_location(sink_block, netlist);

        // TODO: Obter nó RR correspondente ao pin
        RRNodeId sink_node;
        if (rr_graph.nodes().size() > 0) {
            auto nodes = rr_graph.nodes();
            auto sink_nodes_it = nodes.begin();
            std::advance(sink_nodes_it, sink_pins.size() % nodes.size());
            sink_node = *sink_nodes_it;
        }

        if (sink_node.is_valid()) {
            sink_locations.push_back(sink_loc);
            sink_pins.push_back(sink_pin);
            sink_nodes.push_back(sink_node);
        }
    }

    if (sink_nodes.empty()) {
        return result;
    }

    // Encontrar ponto de Steiner
    Point steiner_point = find_steiner_point(source_location, sink_locations, 1.0f);

    // Encontrar nó RRGraph mais próximo do ponto de Steiner
    RRNodeId steiner_node = find_nearest_rr_node(steiner_point, rr_graph, CHANX);
    if (!steiner_node.is_valid()) {
        steiner_node = source_node;  // Fallback
    }

    // Usar path finder para rotear
    PathFinder path_finder(rr_graph, congestion_map_);

    // Conectar cada sink
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

    std::cout << "Starting routing of " << result.total_nets << " nets...\n";

    // Rotear cada net
    for (ClusterNetId net_id : netlist.nets()) {
        NetRoutingResult net_result = route_net(net_id, netlist, rr_graph);

        if (net_result.success) {
            result.nets_routed++;
        }

        result.net_results.push_back(net_result);

        // Progress
        if ((result.net_results.size() % 100) == 0) {
            std::cout << "Routed " << result.net_results.size() << "/" << result.total_nets << " nets\n";
        }
    }

    return result;
}

} // namespace routing

