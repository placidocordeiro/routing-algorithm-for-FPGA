cmake_minimum_required(VERSION 3.16)
project(routing_fpga CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ==============================================================================
# Caminhos base do VTR
# ==============================================================================

set(VTR_ROOT  "${CMAKE_SOURCE_DIR}/vtr")
set(VTR_BUILD "${VTR_ROOT}/build")
set(VTR_LIBS  "${VTR_BUILD}/libs")
set(VTR_EXT   "${VTR_LIBS}/EXTERNAL")

# ==============================================================================
# Headers
# ==============================================================================

set(VTR_INCLUDE_DIRS
    # Arquitetura
    "${VTR_ROOT}/libs/libarchfpga/src"

    # Utilitários
    "${VTR_ROOT}/libs/libvtrutil/src"
    "${VTR_ROOT}/libs/libpugiutil/src"
    "${VTR_ROOT}/libs/EXTERNAL/libpugixml/src"
    "${VTR_ROOT}/libs/EXTERNAL/libargparse/src"
    "${VTR_ROOT}/vpr/src/pack"
    "${VTR_ROOT}/libs/EXTERNAL/libtatum/libtatum/"
    "${VTR_ROOT}/vpr/src/power/"
    "${VTR_ROOT}/vpr/src/noc/"
    "${VTR_ROOT}/vpr/src/server/"
    "${VTR_ROOT}/libs/EXTERNAL/sockpp/include/"
    "${VTR_ROOT}/vpr/src/timing/"

    # Grafo de roteamento
    "${VTR_ROOT}/libs/librrgraph/src/base"

    # Netlist
    "${VTR_ROOT}/libs/libnetlist/src"

    # VPR (API, tipos, contexto global)
    "${VTR_ROOT}/vpr/src/base"
    "${VTR_ROOT}/vpr/src/route"
    "${VTR_ROOT}/vpr/src/place"
    "${VTR_ROOT}/vpr/src/util"

    # Headers gerados pelo build
    "${VTR_BUILD}/libs/libvtrutil"
    "${VTR_BUILD}/vpr"
)

# ==============================================================================
# Libs pré-compiladas (compiladas uma única vez em vtr/build)
# ==============================================================================

set(VTR_LIB_VPR       "${VTR_BUILD}/vpr/libvpr.a")
set(VTR_LIB_RRGRAPH   "${VTR_LIBS}/librrgraph/librrgraph.a")
set(VTR_LIB_ARCHFPGA  "${VTR_LIBS}/libarchfpga/libarchfpga.a")
set(VTR_LIB_VTRUTIL   "${VTR_LIBS}/libvtrutil/libvtrutil.a")
set(VTR_LIB_PUGIUTIL  "${VTR_LIBS}/libpugiutil/libpugiutil.a")
set(VTR_LIB_PUGIXML   "${VTR_EXT}/libpugixml/libpugixml.a")
set(VTR_LIB_LOG       "${VTR_LIBS}/liblog/liblog.a")
set(VTR_LIB_ARGPARSE  "${VTR_EXT}/libargparse/libargparse.a")
set(VTR_LIB_TATUM     "${VTR_EXT}/libtatum/libtatum/libtatum.a")
set(VTR_LIB_CAPNP     "${VTR_EXT}/capnproto/c++/src/capnp/libcapnp.a")
set(VTR_LIB_KJ        "${VTR_EXT}/capnproto/c++/src/kj/libkj.a")
set(VTR_LIB_CAPNPROTO "${VTR_LIBS}/libvtrcapnproto/liblibvtrcapnproto.a")
set(VTR_LIB_SDCPARSE  "${VTR_EXT}/libsdcparse/libsdcparse.a")
set(VTR_LIB_BLIFPARSE "${VTR_EXT}/libblifparse/libblifparse.a")

set(VTR_ALL_LIBS
    ${VTR_LIB_VPR}
    ${VTR_LIB_RRGRAPH}
    ${VTR_LIB_ARCHFPGA}
    ${VTR_LIB_VTRUTIL}
    ${VTR_LIB_PUGIUTIL}
    ${VTR_LIB_PUGIXML}
    ${VTR_LIB_LOG}
    ${VTR_LIB_ARGPARSE}
    ${VTR_LIB_TATUM}
    ${VTR_LIB_CAPNPROTO}
    ${VTR_LIB_CAPNP}
    ${VTR_LIB_KJ}
    ${VTR_LIB_SDCPARSE}
    ${VTR_LIB_BLIFPARSE}
)
# Verifica existência de todas as libs antes de prosseguir
foreach(lib ${VTR_ALL_LIBS})
    if(NOT EXISTS ${lib})
        message(FATAL_ERROR
            "Lib não encontrada: ${lib}\n"
            "Compile o VTR primeiro:\n"
            "  cd vtr/build\n"
            "  make vpr librrgraph libarchfpga libvtrutil libpugiutil liblog -j$(nproc)"
        )
    endif()
endforeach()

# ==============================================================================
# Dependências do sistema
# ==============================================================================

find_package(Threads REQUIRED)

# ==============================================================================
# brkgaAPI — disponível para uso futuro, não utilizado pelo fpga_router
# ==============================================================================

find_package(OpenMP QUIET)
if(OpenMP_CXX_FOUND)
    add_library(brkga STATIC
        extern/brkgaAPI/brkgaAPI/Population.cpp
    )
    target_include_directories(brkga PUBLIC
        "${CMAKE_SOURCE_DIR}/extern/brkgaAPI/brkgaAPI"
    )
    target_link_libraries(brkga PUBLIC OpenMP::OpenMP_CXX)
endif()

# ==============================================================================
# Targets
# ==============================================================================

add_executable(fpga_router
    src/main.cpp
    src/vpr_context/loader.cpp
    src/routing/geometry_utils.cpp
    src/routing/path_finder.cpp
    src/routing/steiner_router.cpp
    src/routing/route_exporter.cpp
)

target_include_directories(fpga_router PRIVATE
    ${VTR_INCLUDE_DIRS}
    "${CMAKE_SOURCE_DIR}/include"    # permite #include "routing/..." headers
    "${CMAKE_SOURCE_DIR}/src"        # permite #include "vpr_context/loader.h"
)

target_link_libraries(fpga_router PRIVATE
    ${VTR_ALL_LIBS}
    Threads::Threads
    dl
    z
)

# ==============================================================================
# ilp_router — modelo ILP de roteamento (CPLEX)
# ==============================================================================

set(CPLEX_ROOT "/opt/ibm/ILOG/CPLEX_Studio2211" CACHE PATH "Raiz da instalação do CPLEX Studio")

if(EXISTS "${CPLEX_ROOT}")
    set(CPLEX_LIB_DIR   "${CPLEX_ROOT}/cplex/lib/x86-64_linux/static_pic")
    set(CONCERT_LIB_DIR "${CPLEX_ROOT}/concert/lib/x86-64_linux/static_pic")

    add_executable(ilp_router
        src/ilp_main.cpp
        src/vpr_context/loader.cpp
        src/ilp/ilp_router.cpp
    )

    target_include_directories(ilp_router PRIVATE
        ${VTR_INCLUDE_DIRS}
        "${CMAKE_SOURCE_DIR}/include"
        "${CMAKE_SOURCE_DIR}/src"
        "${CPLEX_ROOT}/cplex/include"
        "${CPLEX_ROOT}/concert/include"
    )

    target_compile_definitions(ilp_router PRIVATE IL_STD)

    target_link_libraries(ilp_router PRIVATE
        ${VTR_ALL_LIBS}
        "${CPLEX_LIB_DIR}/libilocplex.a"
        "${CONCERT_LIB_DIR}/libconcert.a"
        "${CPLEX_LIB_DIR}/libcplex.a"
        Threads::Threads
        dl
        z
        m
    )
else()
    message(STATUS "CPLEX não encontrado em ${CPLEX_ROOT} — target ilp_router desabilitado")
endif()
