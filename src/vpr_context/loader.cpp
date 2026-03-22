#include "loader.h"
#include <vector>

void load_vpr_context(
    const char* arch_file,
    const char* blif_file,
    const char* net_file,
    const char* place_file,
    t_options&   options,
    t_vpr_setup& vpr_setup,
    t_arch&      arch
) {
    std::vector<const char*> vpr_argv = {
        "vpr",
        arch_file,
        "circuit",
        "--circuit_file",    blif_file,
        "--net_file",        net_file,
        "--place_file",      place_file,
        "--route_file",      "dummy.route",
        "--timing_analysis", "off",
        "--disp",            "off",
    };

    vpr_initialize_logging();
    vpr_init((int)vpr_argv.size(), vpr_argv.data(), &options, &vpr_setup, &arch);
    vpr_load_packing(vpr_setup, arch);
    vpr_create_device(vpr_setup, arch, /*is_flat=*/false);
    vpr_load_placement(vpr_setup, arch);
}