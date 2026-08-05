# Análise de Delay no Roteador ILP

Data: 28/07/2026

## Problema

O modelo ILP atual minimiza apenas `base_cost` (custo estático de congestionamento do PathFinder). Delay não participa da formulação — é medido somente pós-roteamento via Elmore nas `route_trees` reconstruídas. Consequência: soluções ótimas no objetivo podem ter delay pior que o VTR (ex: `mult_5x5/w_min` soma 5,3% maior, `reg_4x32/w_1.3x` delay máximo 5,9% pior).

---

## 1. Como o VTR calcula delay

### Modelo: Elmore em árvore RC

O delay de cada net é o **Elmore delay** da árvore de roteamento (route tree), calculado top-down a partir do source.

**Componentes por nó da route tree:**

| Parâmetro | Origem | Acesso |
|-----------|--------|--------|
| `R` (resistência do nó) | `t_rr_rc_data` | `rr_graph.node_R(inode)` |
| `C` (capacitância do nó) | `t_rr_rc_data` | `rr_graph.node_C(inode)` |
| `R_switch` (resistência do switch) | `t_rr_switch_inf` | `rr_switch_inf[iswitch].R` |
| `Tdel_switch` (delay intrínseco) | `t_rr_switch_inf` | `rr_switch_inf[iswitch].Tdel` |
| `Cinternal` (capacitância interna mux) | `t_rr_switch_inf` | `rr_switch_inf[iswitch].Cinternal` |
| `buffered` (se switch é buffer) | `t_rr_switch_inf` | `rr_switch_inf[iswitch].buffered()` |

**Arquivos VTR:**
- `extern/vtr/libs/libarchfpga/src/physical_types.h:1811-1841` — `t_rr_switch_inf`
- `extern/vtr/libs/librrgraph/src/base/rr_node_types.h:117-122` — `t_rr_rc_data`
- `extern/vtr/libs/librrgraph/src/base/rr_graph_view.h:171-181` — `node_R()`, `node_C()`

### Fórmula Elmore (route tree)

**Arquivo:** `extern/vtr/vpr/src/route/route_tree.cpp:327-348`

```
Para cada nó da árvore (top-down, a partir do source):

  Tdel(nó) = Tarrival + 0.5 * C_downstream(nó) * R(nó)

Para cada filho via switch:
  Tarrival(filho) = Tdel(pai) + R_switch * C_downstream(filho) + Tdel_switch
```

Onde:
- `R_upstream` = soma das resistências do source até o nó (resets em switches buffered)
- `C_downstream` = soma das capacitâncias da subárvore abaixo do nó

### Delay total de uma conexão (timing analysis)

**Arquivo:** `extern/vtr/vpr/src/timing/PostClusterDelayCalculator.tpp:193-381`

```
edge_delay = driver_clb_delay + inter_cluster_delay + sink_clb_delay
```

Onde `inter_cluster_delay = net_delay[net_id][sink_pin]` (Elmore da route tree).

**Nota:** O ILP atual desativa timing analysis (`--timing_analysis off` em `src/vpr_context/loader.cpp:21`), mas `load_net_delay_from_routing()` ainda funciona porque opera diretamente sobre a route tree, sem depender do STA.

---

## 2. Como o ILP mede delay hoje

### Fluxo atual

```
1. VTR roteia → route_trees populadas
2. load_net_delay_from_routing() → vtr_delay[net][pin]     (ANTES do ILP)
3. ILP resolve (objetivo = Σ base_cost * x)
4. Solução ILP → walkback_path() → mirror_path_to_vtr()   (reconstrói route_trees)
5. load_net_delay_from_routing() → ilp_delay[net][pin]     (DEPOIS do ILP)
6. print_delay_comparison() → tabela comparativa
```

**Arquivo:** `src/ilp/ilp_router.cpp`

| Etapa | Linhas | Descrição |
|-------|--------|-----------|
| Captura delay VTR | 572-574 | `load_net_delay_from_routing()` antes do modelo |
| Objetivo ILP | 677-687 | `min Σ base_cost(v) * x[i][v]` — **sem delay** |
| Reconstrução trees | 945-961 | `mirror_path_to_vtr()` para cada net/sink |
| Captura delay ILP | 963-964 | `load_net_delay_from_routing()` após espelhamento |
| Comparação | 966 | `print_delay_comparison()` |

### Métricas comparadas (print_delay_comparison)

**Arquivo:** `src/ilp/ilp_router.cpp:316-361`

- Max delay por net (pior sink)
- Max delay global
- Soma de todos os sinks
- Média por sink
- Razão ILP/VTR para cada métrica

---

## 3. Dados de delay acessíveis no momento do build do modelo

No ponto onde o modelo ILP é construído (linha ~656 em diante), os seguintes dados estão disponíveis via `g_vpr_ctx`:

### 3a. Por nó do RR graph

```cpp
const auto& rr_graph = g_vpr_ctx.device().rr_graph;

float R = rr_graph.node_R(RRNodeId(v));    // resistência do nó
float C = rr_graph.node_C(RRNodeId(v));    // capacitância do nó
int type = rr_graph.node_type(RRNodeId(v)); // SOURCE/SINK/CHANX/CHANY/IPIN/OPIN
```

### 3b. Por aresta do RR graph

```cpp
for (t_edge_size i = 0; i < rr_graph.num_edges(node); i++) {
    short sw = rr_graph.edge_switch(node, i);          // índice do switch
    float R_sw = rr_switch_inf[sw].R;                  // resistência
    float Tdel_sw = rr_switch_inf[sw].Tdel;            // delay intrínseco
    float Cinternal = rr_switch_inf[sw].Cinternal;     // capacitância interna
    bool buffered = rr_switch_inf[sw].buffered();       // se é buffered
}
```

### 3c. Delay por segmento (indexed data)

```cpp
auto& indexed = rr_graph.rr_indexed_data(cost_index);
float T_linear = indexed.T_linear;        // delay linear por segmento
float T_quadratic = indexed.T_quadratic;  // delay quadrático (unbuffered)
float C_load = indexed.C_load;            // capacitância de carga por segmento
```

**Arquivo:** `extern/vtr/libs/librrgraph/src/base/rr_node.h:141-150`

### 3d. Delay do roteamento VTR (warm start)

```cpp
// Já capturado na linha 572-574:
NetPinsMatrix<float> vtr_delay;  // delay Elmore de cada sink de cada net
```

---

## 4. Por que base_cost ≠ delay

`base_cost` é o custo estático de congestionamento do PathFinder:

```
base_cost(v) = custo fixo do nó, independente da rota escolhida
```

Não reflete:
- **R_upstream acumulado** — um nó pode ter baixo base_cost mas estar no fim de um caminho longo com alta resistência
- **C_downstream** — branching na árvore aumenta o delay de todos os descendentes
- **Switches unbuffered** — delay quadrático (N²) que base_cost não captura
- **Topologia da árvore** — duas rotas com mesmos nós podem ter delays diferentes dependendo da estrutura da árvore

O Elmore delay é **não-linear** e **dependente da árvore** (não só dos nós individuais), o que torna sua incorporação direta no ILP desafiadora.

---

## 5. Abordagens para incorporar delay no modelo

### 5a. Aproximação linear por aresta (mais simples)

Atribuir um custo de delay fixo por aresta do RR graph:

```
delay_aresta(u→v) ≈ Tdel_switch + R_switch * C(v) + 0.5 * R(v) * C(v)
```

Objetivo ponderado:
```
min α * Σ base_cost(v) * x[i][v] + β * Σ delay_aresta(e) * y[i][e]
```

Onde `y[i][e] ∈ {0,1}` indica se a aresta `e` é usada pela net `i`.

**Problema:** ignora R_upstream acumulado e C_downstream (efeitos de árvore). É uma aproximação grosseira — funciona bem para switches buffered (onde R_upstream reseta), mal para pass transistors.

**Variáveis novas:** `y[i][e]` para cada aresta no domínio de cada net. Pode ser acoplada com `x` via `y[i][e] ≤ x[i][tail]` e `y[i][e] ≤ x[i][head]`.

**Onde obter os dados:**
- `rr_graph.edge_switch(node, i)` → índice do switch
- `rr_switch_inf[sw].Tdel`, `.R`, `.Cinternal`
- `rr_graph.node_R(v)`, `rr_graph.node_C(v)`

### 5b. Delay por caminho source→sink (formulação com variáveis de caminho)

Para cada sink `t` da net `i`, criar variáveis de fluxo que acumulam delay ao longo do caminho:

```
d[i][t] = delay do caminho source→sink t da net i
```

Restrições de acumulação (para cada nó `v` no caminho):
```
d_arrival[i][v] = d_arrival[i][parent] + delay_aresta(parent→v)
```

Objetivo:
```
min α * congestionamento + β * max_over_nets(max_sink(d[i][t]))
```

**Problema:** requer linearização do max e variáveis de arrival time por nó por net. O número de variáveis cresce muito. Além disso, `d_arrival` depende de qual nó é o "parent" na árvore, o que é uma decisão do modelo.

### 5c. Objetivo lexicográfico (duas fases)

**Fase 1:** Resolver o ILP atual (min congestionamento) → obter objetivo ótimo `Z*`.

**Fase 2:** Adicionar restrição `congestionamento ≤ Z* + ε` e minimizar delay.

**Vantagem:** não precisa misturar unidades (base_cost vs segundos). Garante que delay não piora o congestionamento.

**Implementação:** duas chamadas ao CPLEX, reutilizando o modelo com restrição adicional.

### 5d. Delay como penalidade no warm start

Usar o delay do VTR como referência. Penalizar desvios:

```
min Σ base_cost(v) * x[i][v] + γ * Σ |delay_ilp[i][t] - delay_vtr[i][t]|
```

**Problema:** `delay_ilp` não é linear nos `x` — depende da estrutura da árvore.

### 5e. Proxy de delay: profundidade/hop count

Minimizar número de arestas no caminho (hop count) como proxy de delay:

```
min α * Σ base_cost(v) * x[i][v] + β * Σ y[i][e]
```

**Vantagem:** linear, simples, não precisa de dados RC. Funciona como regularizador contra caminhos longos.

**Desvantagem:** proxy grosseiro — não diferencia switches buffered vs unbuffered, nem comprimentos de fio.

### 5f. Bounding box de delay (constraint)

Adicionar constraint de delay máximo por net:

```
delay_caminho(source → sink_t) ≤ D_max[i][t]   ∀ net i, sink t
```

Onde `D_max[i][t]` pode ser o delay do VTR * (1 + margem).

**Problema:** mesma dificuldade de linearizar o delay por caminho.

---

## 6. Recomendação de implementação

### Ordem de prioridade

1. **Proxy hop count (5e)** — implementação trivial, baixo custo de variáveis, testar se já melhora os casos problemáticos. Medir impacto em `mult_5x5/w_min` e `reg_4x32/w_1.3x`.

2. **Delay linear por aresta (5a)** — se hop count não for suficiente. Requer dados RC já disponíveis via `rr_graph` e `rr_switch_inf`. Aproximação razoável para arquitetura `k6_frac_N10` (switches majoritariamente buffered nos canais).

3. **Lexicográfico (5c)** — se a ponderação α/β for difícil de calibrar. Duas fases evita misturar unidades.

### Dados a extrair antes do build

Para qualquer abordagem que use delay, coletar antes da construção do modelo:

```cpp
// Para cada aresta global e:
//   delay_e = Tdel_sw + R_sw * C(head) + 0.5 * R(head) * C(head)
//   (ajustar com Cinternal se switch não buffered)

// Para cada net i e sink t:
//   vtr_delay[i][t] — já disponível na linha 572-574
```

### Impacto estimado no tamanho do modelo

| Abordagem | Variáveis novas | Restrições novas |
|-----------|----------------|-----------------|
| Hop count (5e) | `y[i][e]` por aresta/net | `y ≤ x_tail`, `y ≤ x_head` por aresta/net |
| Delay aresta (5a) | mesmas `y[i][e]` | mesmas |
| Lexicográfico (5c) | nenhuma | 1 constraint adicional (2ª fase) |
| Arrival time (5b) | `d_arrival[i][v]` por nó/net | conservação de arrival por nó/net |

---

## 7. Casos onde delay ILP > delay VTR (alvos para validação)

| Circuito | Config | Razão max delay | Razão soma delay |
|----------|--------|-----------------|-----------------|
| mult_5x5 | w_min | 1.000 | **1.053** |
| reg_4x32 | w_1.3x | **1.059** | 0.993 |
| reg_4x32 | w_min | 0.834 | 0.972 |

Estes são os casos onde a incorporação de delay no objetivo deve trazer melhoria mensurável.
