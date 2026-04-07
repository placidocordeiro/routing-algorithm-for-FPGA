#!/usr/bin/env bash
# Executa fpga_router para todas as combinações de arquitetura x circuito.
# Uso: ./run_all.sh [--build]   (--build recompila antes de rodar)

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

ROUTER="$SCRIPT_DIR/build/fpga_router"
ARCH_DIR="$SCRIPT_DIR/arch"
BENCH_DIR="$SCRIPT_DIR/benchmarks"

# Recompilar se pedido
if [[ "${1}" == "--build" ]]; then
    echo ">>> Compilando..."
    cmake -S "$SCRIPT_DIR" -B "$SCRIPT_DIR/build" -DCMAKE_BUILD_TYPE=Release -Wno-dev > /dev/null
    make -C "$SCRIPT_DIR/build" -j"$(nproc)" fpga_router
    echo ""
fi

if [[ ! -x "$ROUTER" ]]; then
    echo "ERRO: $ROUTER não encontrado. Compile com: cmake .. && make"
    exit 1
fi

# Coletar arquiteturas e circuitos
mapfile -t ARCHS < <(find "$ARCH_DIR" -name "*.xml" | sort)
mapfile -t CIRCUITS < <(find "$BENCH_DIR" -name "*.net" | sort)

total=$(( ${#ARCHS[@]} * ${#CIRCUITS[@]} ))
echo "Arquiteturas : ${#ARCHS[@]}"
echo "Circuitos    : ${#CIRCUITS[@]}"
echo "Combinações  : $total"
echo ""

idx=0
failed=0

for arch in "${ARCHS[@]}"; do
    arch_name=$(basename "$arch" .xml)

    for net in "${CIRCUITS[@]}"; do
        dir=$(dirname "$net")
        base=$(basename "$net" .net)

        blif="$dir/${base}.pre-vpr.blif"
        place="$dir/${base}.place"

        if [[ ! -f "$blif" || ! -f "$place" ]]; then
            echo "[SKIP] $base — arquivos incompletos"
            continue
        fi

        idx=$(( idx + 1 ))
        printf "[%3d/%d] %-30s x %s\n" "$idx" "$total" "$base" "$arch_name"

        "$ROUTER" "$arch" "$blif" "$net" "$place" 2>&1 \
            || { echo "  !! FALHOU"; failed=$(( failed + 1 )); }

        echo ""
    done
done

echo "=============================="
echo "Concluído: $idx combinações"
[[ $failed -gt 0 ]] && echo "Falhas: $failed" || echo "Todas OK"
