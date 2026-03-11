#!/bin/bash

# Auto-setup script para VTR e Yosys caso queira rodar localmente e gerar as netlists 
# para algum circuito específico. O script faz o seguinte:

# 1. Verifica se VTR/Yosys estão instalados
# 2. Se não, baixa e compila
# 3. Configura PATH

VTR_DIR="tools/vtr"
YOSYS_DIR="tools/yosys"

# Clona e compila se necessário
if [ ! -d "$VTR_DIR" ]; then
    git clone --depth 1 https://github.com/verilog-to-routing/vtr-verilog-to-routing.git $VTR_DIR
    cd $VTR_DIR && mkdir build && cd build
    cmake .. -DCMAKE_BUILD_TYPE=Release
    make -j4 vpr yosys
fi

# Adiciona ao PATH localmente
export PATH="$(pwd)/$VTR_DIR/build/bin:$(pwd)/$VTR_DIR/build/vpr:$PATH"