#ifndef VALIDATION_VALIDATE_H
#define VALIDATION_VALIDATE_H

#include "../routing/types.h"
#include "../netlist/types.h"
#include <vector>
#include <map>

struct ValidationStats {
    bool is_valid;
    int overused_nodes;
    int disconnected_paths;
    int unrouted_nets;
};

class Validator {
public:
    // Valida conectividade e capacidade
    static ValidationStats validateRouting(
        const RoutingGraph& graph,
        const std::vector<RouteTree>& routes,
        const std::vector<Net>& physical_nets
    );
};

#endif