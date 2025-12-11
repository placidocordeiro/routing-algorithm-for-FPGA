#ifndef ARCHITECTURE_TYPES_H
#define ARCHITECTURE_TYPES_H

#include <string>
#include <vector>

struct Switch {
    std::string type, name;
    double R, Cin, Cout, Tdel, mux_trans_size, buf_size;
};

struct Segment {
    double freq;
    int length;
    std::string type;
    double Rmetal, Cmetal;
    std::string mux_name;
};

struct Direct {
    std::string name, from_pin, to_pin;
    int x_offset, y_offset, z_offset;
};

struct Port {
    std::string name, type;
    int num_pins;
    bool is_clock;
};

struct Tile {
    std::string name, type;
    int height;
    double area, fc_in, fc_out;
    std::vector<Port> ports;
};

struct Device {
    double R_minW_nmos, R_minW_pmos, grid_logic_tile_area;
    std::string switch_block_type, connection_block_switch;
    int fs;
};

struct FPGAArchitecture {
    Device device;
    std::vector<Switch> switches;
    std::vector<Segment> segments;
    std::vector<Direct> directs;
    std::vector<Tile> tiles;
};

#endif