cmake_minimum_required(VERSION 3.12)
project(FPGARouter VERSION 1.0.0 LANGUAGES CXX)

# C++ Standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Compiler flags
if(MSVC)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /W4")
else()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic -O2")
endif()

# Find required packages
find_package(tinyxml2 REQUIRED)

include_directories(${CMAKE_SOURCE_DIR}/include)

# Source files
set(SOURCES
    src/main.cpp
    src/architecture/parser.cpp
    src/netlist/parser.cpp
    src/placement/parser.cpp
    src/routing/graph_builder.cpp
)

# Main executable
add_executable(fpga_router ${SOURCES})

target_link_libraries(fpga_router PRIVATE tinyxml2::tinyxml2)

# Output directory
set_target_properties(fpga_router PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)

# Create output directories
file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

install(TARGETS fpga_router RUNTIME DESTINATION bin)

# Print configuration info
message(STATUS "=== FPGA Router Configuration ===")
message(STATUS "Project: ${PROJECT_NAME}")
message(STATUS "Version: ${PROJECT_VERSION}")
message(STATUS "C++ Standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "tinyxml2 found: ${tinyxml2_FOUND}")
message(STATUS "Output directory: ${CMAKE_BINARY_DIR}/bin")
message(STATUS "====================================")