# Implementação do objetivo de delay

Data: 28/07/2026

## Status

Implementado em `src/ilp/ilp_router.cpp`. O programa não foi executado; a
validação funcional dos benchmarks permanece manual.

## Decisão

O modelo passou a usar dois objetivos lexicográficos:

1. minimizar `base_cost`, sem mudar a prioridade do objetivo original;
2. entre soluções de mesmo `base_cost`, minimizar um proxy RC de delay.

Foi reutilizada a variável de fluxo contínua `f[i][e]`, já existente. Isso evita
criar uma variável binária por aresta/net e não adiciona restrições. Como o fluxo
de uma aresta corresponde ao número de sinks atendidos a jusante, o segundo
objetivo aproxima a soma dos delays source→sink, incluindo o efeito do
compartilhamento de ramos.

## Formulação

Para cada aresta `e = u→v`:

```text
proxy_e =
    Tdel_switch
  + R_switch * (C(v) + Cinternal_switch)
  + 0,5 * R(v) * C(v)
```

O valor é convertido para nanossegundos antes de ser entregue ao CPLEX.

```text
min lexicograficamente:
    1. Σ base_cost(v) * x[i][v]
    2. Σ proxy_e * f[i][e]
```

Foi usado `IloStaticLex`, com `base_cost` como critério primário. Portanto, delay
não pode comprar uma solução de congestionamento pior apenas por ter menor proxy.

## Alterações realizadas

- Cada aresta global agora armazena:
  - origem e destino;
  - `RREdgeId` exato;
  - proxy RC em ns.
- O warm start do VTR passou a calcular e reportar `base_cost` e proxy RC.
- O relatório do solver passou a mostrar:
  - valor do objetivo primário;
  - valor do proxy RC;
  - quantidade de subproblemas multiobjetivo;
  - melhor bound e gap do objetivo primário.
- A proteção de qualidade continua comparando o `base_cost` do ILP com o warm
  start VTR. Se o ILP for pior, a route tree original é preservada.
- A reconstrução da rota agora segue somente arestas com fluxo positivo.
  Anteriormente, o BFS usava qualquer aresta entre nós com `x = 1`, podendo
  exportar um caminho diferente daquele avaliado pelo objetivo.
- O `RREdgeId` é preservado para tratar corretamente arestas paralelas com
  switches diferentes.

## Por que esta abordagem

- Aproveita variáveis já presentes no modelo.
- Mantém o tamanho do ILP: nenhuma variável ou restrição nova.
- Usa dados elétricos reais do RRGraph, sendo mais informativa que hop count.
- Evita calibrar pesos entre unidades incompatíveis.
- Preserva o objetivo e o warm start existentes.
- Mantém o Elmore exato como medição pós-roteamento, sem confundir proxy de
  otimização com delay final.

## Limitações

O proxy é linear e local. Ele não representa exatamente `R_upstream`,
`C_downstream` nem toda a topologia RC da árvore. A avaliação definitiva continua
sendo `load_net_delay_from_routing()` após reconstruir as route trees.

Se o time limit terminar antes de o CPLEX concluir o objetivo primário, o segundo
nível lexicográfico pode não ser otimizado. O relatório indica quantos
subproblemas foram resolvidos.

## Validação realizada

- Compilação de `src/ilp/ilp_router.cpp` com:
  `-Wall -Wextra -Wpedantic`: sem erros e sem warnings.
- Compilação dos quatro fontes do executável atual: concluída.
- Linkedição completa com VTR e CPLEX 22.1: concluída.
- `git diff --check`: sem problemas.
- O executável não foi rodado.

Warnings emitidos durante a linkedição vieram de objetos LTO pré-compilados do
VTR e não dos fontes alterados.

## Testes manuais recomendados

Comparar o novo relatório com os resultados anteriores:

| Circuito | Configuração | Métrica principal |
|---|---|---|
| `mult_5x5` | `w_min` | soma dos delays |
| `reg_4x32` | `w_1.3x` | delay máximo |
| `reg_4x32` | `w_min` | controle de regressão |

Verificar em cada execução:

1. todas as nets conectadas no BFS;
2. `base_cost` ILP não pior que o warm start VTR;
3. segundo subproblema multiobjetivo alcançado;
4. redução do proxy RC;
5. redução ou manutenção do delay Elmore real;
6. ausência de regressão relevante no tempo de solve.
