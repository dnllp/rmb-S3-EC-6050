// Pruebas del planificador en la PC:  pio test -e native
#include <unity.h>

#include <algorithm>
#include <queue>
#include <random>

#include "Planner.h"

using namespace planner;

static SearchParams P(Algorithm a, float cg = 0.5f, Heuristic h = Heuristic::MANHATTAN) {
  SearchParams p;
  p.algo = a;
  p.turnCost = cg;
  p.heuristic = h;
  return p;
}

// BFS de referencia, escrito aparte: distancia mínima en celdas (−1 si no hay ruta).
static int refDistance(const Grid &g, Cell a, Cell b) {
  if (g.isBlocked(a) || g.isBlocked(b)) return -1;
  int dist[NCELLS];
  for (int &d : dist) d = -1;
  std::queue<Cell> q;
  dist[cellIndex(a)] = 0;
  q.push(a);
  while (!q.empty()) {
    Cell k = q.front();
    q.pop();
    for (uint8_t h = 0; h < 4; h++) {
      Cell n = step(k, h);
      if (g.isFree(n) && dist[cellIndex(n)] < 0) {
        dist[cellIndex(n)] = dist[cellIndex(k)] + 1;
        q.push(n);
      }
    }
  }
  return dist[cellIndex(b)];
}

// Una ruta es válida si empieza y termina donde debe, avanza entre vecinos y no pisa obstáculos.
static void assertValidPath(const Grid &g, const SearchResult &r, Cell a, Cell b) {
  TEST_ASSERT_TRUE(r.found);
  TEST_ASSERT_TRUE(r.path.front() == a);
  TEST_ASSERT_TRUE(r.path.back() == b);
  for (size_t i = 0; i < r.path.size(); i++) {
    TEST_ASSERT_TRUE(g.isFree(r.path[i]));
    if (i) TEST_ASSERT_NOT_EQUAL(255, directionBetween(r.path[i - 1], r.path[i]));
  }
}

// ---------------------------------------------------------------- mapas
static Grid emptyMap() { return Grid(); }

// Pasillo: fila 2 libre, filas 1 y 3 bloqueadas de c=1 a c=4
static Grid corridorMap() {
  Grid g;
  for (int c = 1; c <= 4; c++) {
    g.setBlocked(Cell(1, c), true);
    g.setBlocked(Cell(3, c), true);
  }
  return g;
}

// U: paredes en c=1 y c=4 de f=1 a f=4, fondo en f=1 (c=1..4). Hay que rodearla.
static Grid uMap() {
  Grid g;
  for (int f = 1; f <= 4; f++) {
    g.setBlocked(Cell(f, 1), true);
    g.setBlocked(Cell(f, 4), true);
  }
  for (int c = 1; c <= 4; c++) g.setBlocked(Cell(1, c), true);
  return g;
}

// Meta (5,5) encerrada por (4,5) y (5,4)
static Grid enclosedMap() {
  Grid g;
  g.setBlocked(Cell(4, 5), true);
  g.setBlocked(Cell(5, 4), true);
  return g;
}

static Grid randomMap(std::mt19937 &rng, float density) {
  Grid g;
  std::uniform_real_distribution<float> u(0, 1);
  for (int i = 0; i < NCELLS; i++)
    if (u(rng) < density) g.setBlocked(cellFromIndex(i), true);
  return g;
}

// ---------------------------------------------------------------- pruebas
void test_mapa_vacio_bfs_minimo_de_celdas() {
  Grid g = emptyMap();
  SearchResult r = search(g, Cell(0, 0), E, Cell(5, 5), P(Algorithm::BFS));
  assertValidPath(g, r, Cell(0, 0), Cell(5, 5));
  TEST_ASSERT_EQUAL(10, r.cells);
}

void test_mapa_vacio_astar_un_solo_giro() {
  Grid g = emptyMap();
  SearchResult r = search(g, Cell(0, 0), E, Cell(5, 5), P(Algorithm::ASTAR));
  assertValidPath(g, r, Cell(0, 0), Cell(5, 5));
  TEST_ASSERT_EQUAL(10, r.cells);
  TEST_ASSERT_EQUAL(1, r.turns);
  TEST_ASSERT_FLOAT_WITHIN(1e-4, 10.5f, r.cost);
}

void test_pasillo() {
  Grid g = corridorMap();
  for (Algorithm a : {Algorithm::BFS, Algorithm::DFS, Algorithm::UCS, Algorithm::ASTAR}) {
    SearchResult r = search(g, Cell(2, 0), E, Cell(2, 5), P(a));
    assertValidPath(g, r, Cell(2, 0), Cell(2, 5));
  }
  SearchResult r = search(g, Cell(2, 0), E, Cell(2, 5), P(Algorithm::ASTAR));
  TEST_ASSERT_EQUAL(5, r.cells);
  TEST_ASSERT_EQUAL(0, r.turns);
}

void test_laberinto_en_u() {
  Grid g = uMap();
  Cell a(3, 2), b(3, 5);  // de dentro de la U a fuera
  int d = refDistance(g, a, b);
  TEST_ASSERT_TRUE(d > 3);
  SearchResult bfs = search(g, a, N, b, P(Algorithm::BFS));
  assertValidPath(g, bfs, a, b);
  TEST_ASSERT_EQUAL(d, bfs.cells);
  SearchResult ucs = search(g, a, N, b, P(Algorithm::UCS));
  SearchResult ast = search(g, a, N, b, P(Algorithm::ASTAR));
  assertValidPath(g, ucs, a, b);
  assertValidPath(g, ast, a, b);
  TEST_ASSERT_FLOAT_WITHIN(1e-4, ucs.cost, ast.cost);
  TEST_ASSERT_TRUE(ast.expanded.size() <= ucs.expanded.size());
}

void test_meta_encerrada() {
  Grid g = enclosedMap();
  for (Algorithm a : {Algorithm::BFS, Algorithm::DFS, Algorithm::UCS, Algorithm::ASTAR}) {
    SearchResult r = search(g, Cell(0, 0), E, Cell(5, 5), P(a));
    TEST_ASSERT_FALSE(r.found);
  }
}

void test_meta_en_obstaculo_es_inalcanzable() {
  Grid g;
  g.setBlocked(Cell(3, 3), true);
  SearchResult r = search(g, Cell(0, 0), E, Cell(3, 3), P(Algorithm::ASTAR));
  TEST_ASSERT_FALSE(r.found);
}

void test_inicio_igual_a_meta() {
  Grid g;
  SearchResult r = search(g, Cell(2, 2), N, Cell(2, 2), P(Algorithm::ASTAR));
  TEST_ASSERT_TRUE(r.found);
  TEST_ASSERT_EQUAL(0, r.cells);
  TEST_ASSERT_FLOAT_WITHIN(1e-6, 0.0f, r.cost);
}

// 300 mapas aleatorios: BFS mínimo de celdas, UCS = A* (ambas heurísticas),
// A* expande ≤ UCS, DFS encuentra ruta si existe.
void test_mapas_aleatorios_propiedades() {
  std::mt19937 rng(12345);
  std::uniform_int_distribution<int> cell(0, NCELLS - 1), head(0, 3);
  std::uniform_real_distribution<float> cgd(0.0f, 2.0f);
  int compared = 0;
  for (int it = 0; it < 300; it++) {
    Grid g = randomMap(rng, 0.25f);
    Cell a = cellFromIndex(cell(rng)), b = cellFromIndex(cell(rng));
    g.setBlocked(a, false);
    g.setBlocked(b, false);
    uint8_t h = (uint8_t)head(rng);
    float cg = cgd(rng);
    int d = refDistance(g, a, b);

    SearchResult bfs = search(g, a, h, b, P(Algorithm::BFS, cg));
    SearchResult dfs = search(g, a, h, b, P(Algorithm::DFS, cg));
    SearchResult ucs = search(g, a, h, b, P(Algorithm::UCS, cg));
    SearchResult a1 = search(g, a, h, b, P(Algorithm::ASTAR, cg, Heuristic::MANHATTAN));
    SearchResult a2 = search(g, a, h, b, P(Algorithm::ASTAR, cg, Heuristic::MANHATTAN_TURNS));

    if (d < 0) {
      TEST_ASSERT_FALSE(bfs.found);
      TEST_ASSERT_FALSE(dfs.found);
      TEST_ASSERT_FALSE(ucs.found);
      TEST_ASSERT_FALSE(a1.found);
      TEST_ASSERT_FALSE(a2.found);
      continue;
    }
    assertValidPath(g, bfs, a, b);
    assertValidPath(g, dfs, a, b);
    assertValidPath(g, ucs, a, b);
    assertValidPath(g, a1, a, b);
    assertValidPath(g, a2, a, b);
    TEST_ASSERT_EQUAL(d, bfs.cells);
    TEST_ASSERT_TRUE(dfs.cells >= bfs.cells);
    TEST_ASSERT_TRUE(ucs.cost <= bfs.cost + 1e-4f);  // UCS optimiza el costo con giros
    TEST_ASSERT_FLOAT_WITHIN(1e-4, ucs.cost, a1.cost);
    TEST_ASSERT_FLOAT_WITHIN(1e-4, ucs.cost, a2.cost);
    TEST_ASSERT_TRUE(a1.expanded.size() <= ucs.expanded.size());
    TEST_ASSERT_TRUE(a2.expanded.size() <= ucs.expanded.size());
    compared++;
  }
  TEST_ASSERT_TRUE(compared > 150);
}

// Fuerza bruta sobre todas las permutaciones
static float bruteForce(const CostMatrix &m, bool ret) {
  std::vector<uint8_t> o;
  for (size_t i = 1; i < m.size(); i++) o.push_back((uint8_t)i);
  float best = INF_COST;
  do best = std::min(best, tourCost(m, o, ret));
  while (std::next_permutation(o.begin(), o.end()));
  return best;
}

void test_held_karp_igual_a_fuerza_bruta() {
  std::mt19937 rng(7);
  std::uniform_real_distribution<float> u(1.0f, 12.0f);
  for (int n = 1; n <= 7; n++) {
    for (int rep = 0; rep < 10; rep++) {
      CostMatrix m(n + 1, std::vector<float>(n + 1, 0));
      for (int i = 0; i <= n; i++)
        for (int j = 0; j <= n; j++)
          if (i != j) m[i][j] = u(rng);
      for (bool ret : {false, true}) {
        float hk;
        std::vector<uint8_t> o = heldKarp(m, ret, &hk);
        TEST_ASSERT_EQUAL(n, (int)o.size());
        float bf = bruteForce(m, ret);
        TEST_ASSERT_FLOAT_WITHIN(1e-3, bf, hk);
        TEST_ASSERT_FLOAT_WITHIN(1e-3, hk, tourCost(m, o, ret));
      }
    }
  }
}

void test_vecino_mas_cercano_2opt_es_valido() {
  std::mt19937 rng(99);
  std::uniform_real_distribution<float> u(1.0f, 12.0f);
  const int n = 12;
  CostMatrix m(n + 1, std::vector<float>(n + 1, 0));
  for (int i = 0; i <= n; i++)
    for (int j = 0; j <= n; j++)
      if (i != j) m[i][j] = u(rng);
  float c;
  std::vector<uint8_t> o = nearestNeighbor2Opt(m, false, &c);
  TEST_ASSERT_EQUAL(n, (int)o.size());
  std::vector<uint8_t> s = o;
  std::sort(s.begin(), s.end());
  for (int i = 0; i < n; i++) TEST_ASSERT_EQUAL(i + 1, s[i]);
  TEST_ASSERT_FLOAT_WITHIN(1e-3, c, tourCost(m, o, false));
}

void test_tour_con_meta_inalcanzable() {
  Grid g = enclosedMap();
  TourParams tp;
  tp.search = P(Algorithm::ASTAR);
  std::vector<Cell> goals = {Cell(2, 2), Cell(5, 5), Cell(0, 5)};
  TourResult t = planTour(g, Cell(0, 0), E, goals, tp);
  TEST_ASSERT_TRUE(t.ok);
  TEST_ASSERT_EQUAL(1, (int)t.unreachable.size());
  TEST_ASSERT_EQUAL(1, t.unreachable[0]);
  TEST_ASSERT_EQUAL(2, (int)t.order.size());
  TEST_ASSERT_EQUAL(2, (int)t.legs.size());
}

void test_tour_orden_optimo_en_linea() {
  // Metas en la fila 0, desordenadas: el orden óptimo es de izquierda a derecha.
  Grid g;
  TourParams tp;
  tp.search = P(Algorithm::UCS);
  std::vector<Cell> goals = {Cell(0, 5), Cell(0, 2), Cell(0, 4), Cell(0, 1)};
  TourResult t = planTour(g, Cell(0, 0), E, goals, tp);
  TEST_ASSERT_TRUE(t.ok);
  TEST_ASSERT_TRUE(t.exact);
  uint8_t expected[] = {3, 1, 2, 0};
  for (int i = 0; i < 4; i++) TEST_ASSERT_EQUAL(expected[i], t.order[i]);
  TEST_ASSERT_FLOAT_WITHIN(1e-4, 5.0f, t.cost);
  TEST_ASSERT_EQUAL(0, t.turns);
}

void test_tour_regresar_al_inicio() {
  Grid g;
  TourParams tp;
  tp.search = P(Algorithm::ASTAR);
  tp.returnHome = true;
  std::vector<Cell> goals = {Cell(0, 3)};
  TourResult t = planTour(g, Cell(0, 0), E, goals, tp);
  TEST_ASSERT_TRUE(t.ok);
  TEST_ASSERT_EQUAL(2, (int)t.legs.size());
  TEST_ASSERT_EQUAL(LEG_RETURN_HOME, t.legs[1].goalIndex);
  TEST_ASSERT_TRUE(t.legs[1].res.path.back() == Cell(0, 0));
  // 3 celdas, giro de 180° (2), 3 celdas
  TEST_ASSERT_FLOAT_WITHIN(1e-4, 7.0f, t.cost);
}

void test_doce_metas_usa_2opt() {
  Grid g;
  TourParams tp;
  tp.search = P(Algorithm::ASTAR);
  std::vector<Cell> goals;
  for (int i = 0; i < 12; i++) goals.push_back(Cell(i / 3 + 1, (i % 3) * 2));
  TourResult t = planTour(g, Cell(0, 0), E, goals, tp);
  TEST_ASSERT_TRUE(t.ok);
  TEST_ASSERT_FALSE(t.exact);
  TEST_ASSERT_EQUAL(12, (int)t.order.size());
}

void test_primitivas_EEEN() {
  std::vector<Cell> path = {Cell(0, 0), Cell(0, 1), Cell(0, 2), Cell(0, 3), Cell(1, 3)};
  std::vector<Primitive> pr;
  appendPathPrimitives(path, E, pr);
  TEST_ASSERT_EQUAL(3, (int)pr.size());
  TEST_ASSERT_EQUAL((int)PrimType::FORWARD, (int)pr[0].type);
  TEST_ASSERT_EQUAL(3, pr[0].cells);
  TEST_ASSERT_EQUAL((int)PrimType::TURN, (int)pr[1].type);
  TEST_ASSERT_EQUAL(90, pr[1].turnDeg);
  TEST_ASSERT_EQUAL(N, pr[1].heading);
  TEST_ASSERT_EQUAL((int)PrimType::FORWARD, (int)pr[2].type);
  TEST_ASSERT_EQUAL(1, pr[2].cells);
  TEST_ASSERT_TRUE(pr[2].target == Cell(1, 3));
}

void test_primitivas_giro_180_y_derecha() {
  // Mirando al E, ir al O: giro de 180°. Luego al N desde O: derecha (−90°).
  std::vector<Cell> path = {Cell(2, 2), Cell(2, 1), Cell(3, 1)};
  std::vector<Primitive> pr;
  appendPathPrimitives(path, E, pr);
  TEST_ASSERT_EQUAL(4, (int)pr.size());
  TEST_ASSERT_EQUAL(180, pr[0].turnDeg);
  TEST_ASSERT_EQUAL(-90, pr[2].turnDeg);
}

void test_tour_a_primitivas_marca_metas() {
  Grid g;
  TourParams tp;
  tp.search = P(Algorithm::ASTAR);
  std::vector<Cell> goals = {Cell(0, 3), Cell(3, 3)};
  TourResult t = planTour(g, Cell(0, 0), E, goals, tp);
  std::vector<Primitive> pr = tourToPrimitives(t, Cell(0, 0), E);
  int marked = 0;
  for (auto &p : pr)
    if (p.goalReached != NO_GOAL) marked++;
  TEST_ASSERT_EQUAL(2, marked);
  TEST_ASSERT_TRUE(pr.back().target == Cell(3, 3));
}

void test_heuristica_giros_admisible() {
  // τ nunca supera los giros reales de una ruta óptima en mapa vacío
  Grid g;
  for (int i = 0; i < NCELLS; i++)
    for (uint8_t h = 0; h < 4; h++) {
      Cell a = cellFromIndex(i), b(5, 5);
      SearchResult r = search(g, a, h, b, P(Algorithm::UCS, 0.5f));
      TEST_ASSERT_TRUE(unavoidableTurns(a, h, b) <= r.turns);
    }
}

void setUp() {}
void tearDown() {}

int main(int, char **) {
  UNITY_BEGIN();
  RUN_TEST(test_mapa_vacio_bfs_minimo_de_celdas);
  RUN_TEST(test_mapa_vacio_astar_un_solo_giro);
  RUN_TEST(test_pasillo);
  RUN_TEST(test_laberinto_en_u);
  RUN_TEST(test_meta_encerrada);
  RUN_TEST(test_meta_en_obstaculo_es_inalcanzable);
  RUN_TEST(test_inicio_igual_a_meta);
  RUN_TEST(test_mapas_aleatorios_propiedades);
  RUN_TEST(test_held_karp_igual_a_fuerza_bruta);
  RUN_TEST(test_vecino_mas_cercano_2opt_es_valido);
  RUN_TEST(test_tour_con_meta_inalcanzable);
  RUN_TEST(test_tour_orden_optimo_en_linea);
  RUN_TEST(test_tour_regresar_al_inicio);
  RUN_TEST(test_doce_metas_usa_2opt);
  RUN_TEST(test_primitivas_EEEN);
  RUN_TEST(test_primitivas_giro_180_y_derecha);
  RUN_TEST(test_tour_a_primitivas_marca_metas);
  RUN_TEST(test_heuristica_giros_admisible);
  return UNITY_END();
}
