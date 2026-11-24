#include "Search.h"

#include <stdlib.h>
#include <string.h>

#ifdef ARDUINO
#include <esp_timer.h>
#else
#include <chrono>
#endif

namespace planner {

uint32_t nowMicros() {
#ifdef ARDUINO
  return (uint32_t)esp_timer_get_time();
#else
  using namespace std::chrono;
  return (uint32_t)duration_cast<microseconds>(steady_clock::now().time_since_epoch()).count();
#endif
}

const char *algorithmName(Algorithm a) {
  switch (a) {
    case Algorithm::BFS: return "bfs";
    case Algorithm::DFS: return "dfs";
    case Algorithm::UCS: return "ucs";
    default: return "astar";
  }
}

bool parseAlgorithm(const char *s, Algorithm &a) {
  if (!s) return false;
  if (!strcmp(s, "bfs")) a = Algorithm::BFS;
  else if (!strcmp(s, "dfs")) a = Algorithm::DFS;
  else if (!strcmp(s, "ucs")) a = Algorithm::UCS;
  else if (!strcmp(s, "astar") || !strcmp(s, "a*")) a = Algorithm::ASTAR;
  else return false;
  return true;
}

float evaluatePath(const std::vector<Cell> &path, uint8_t startHeading, float turnCost,
                   uint16_t *turnsOut, uint8_t *endHeadingOut) {
  uint16_t turns = 0;
  uint8_t h = startHeading;
  for (size_t i = 1; i < path.size(); i++) {
    uint8_t d = directionBetween(path[i - 1], path[i]);
    if (d == 255) return INF_COST;  // ruta inválida
    turns += turnsBetween(h, d);
    h = d;
  }
  if (turnsOut) *turnsOut = turns;
  if (endHeadingOut) *endHeadingOut = h;
  float cells = path.empty() ? 0.0f : (float)(path.size() - 1);
  return cells + turnCost * (float)turns;
}

uint8_t unavoidableTurns(Cell k, uint8_t h, Cell goal) {
  // Direcciones que la ruta tiene que usar sí o sí.
  bool need[4] = {false, false, false, false};
  if (goal.f > k.f) need[N] = true;
  if (goal.f < k.f) need[S] = true;
  if (goal.c > k.c) need[E] = true;
  if (goal.c < k.c) need[O] = true;
  uint8_t count = need[0] + need[1] + need[2] + need[3];
  if (count == 0 || h == ANY) return count == 0 ? 0 : (uint8_t)(count - 1);
  if (need[h]) return (uint8_t)(count - 1);
  if (count == 2) return 2;  // h es opuesto a una y perpendicular a la otra
  // count == 1: perpendicular → 1 giro; opuesta → 2 giros
  uint8_t d = need[N] ? N : need[E] ? E : need[S] ? S : O;
  return turnsBetween(h, d);
}

float heuristic(Cell k, uint8_t h, Cell goal, const SearchParams &p) {
  if (p.algo != Algorithm::ASTAR) return 0.0f;
  float m = (float)(abs(k.f - goal.f) + abs(k.c - goal.c));
  if (p.heuristic == Heuristic::MANHATTAN_TURNS) m += p.turnCost * unavoidableTurns(k, h, goal);
  return m;
}

namespace {

constexpr uint8_t NONE = 255;
constexpr int FRONTIER_CAP = 640;  // inserciones ≤ 4 iniciales + 144 estados × 3 sucesores

struct Entry {
  uint8_t state;
  uint8_t parent;
  float g;
  float f;
  uint16_t seq;  // orden de inserción: desempate estable
};

// Cola FIFO (BFS)
struct FifoFrontier {
  Entry buf[FRONTIER_CAP];
  int head = 0, tail = 0;
  void reset() { head = tail = 0; }
  bool empty() const { return head == tail; }
  bool push(const Entry &e) {
    if (tail >= FRONTIER_CAP) return false;
    buf[tail++] = e;
    return true;
  }
  Entry pop() { return buf[head++]; }
};

// Pila LIFO (DFS)
struct LifoFrontier {
  Entry buf[FRONTIER_CAP];
  int top = 0;
  void reset() { top = 0; }
  bool empty() const { return top == 0; }
  bool push(const Entry &e) {
    if (top >= FRONTIER_CAP) return false;
    buf[top++] = e;
    return true;
  }
  Entry pop() { return buf[--top]; }
};

// Montículo binario por (f, seq) (UCS y A*)
struct PriorityFrontier {
  Entry buf[FRONTIER_CAP];
  int n = 0;
  void reset() { n = 0; }
  static bool less(const Entry &a, const Entry &b) {
    if (a.f != b.f) return a.f < b.f;
    return a.seq < b.seq;
  }
  bool empty() const { return n == 0; }
  bool push(const Entry &e) {
    if (n >= FRONTIER_CAP) return false;
    int i = n++;
    buf[i] = e;
    while (i > 0) {
      int p = (i - 1) / 2;
      if (!less(buf[i], buf[p])) break;
      Entry t = buf[i]; buf[i] = buf[p]; buf[p] = t;
      i = p;
    }
    return true;
  }
  Entry pop() {
    Entry top = buf[0];
    buf[0] = buf[--n];
    int i = 0;
    for (;;) {
      int l = 2 * i + 1, r = l + 1, m = i;
      if (l < n && less(buf[l], buf[m])) m = l;
      if (r < n && less(buf[r], buf[m])) m = r;
      if (m == i) break;
      Entry t = buf[i]; buf[i] = buf[m]; buf[m] = t;
      i = m;
    }
    return top;
  }
};

struct Successor {
  uint8_t state;
  float cost;
};

// Espacio de celdas (BFS, DFS): estado = índice de celda.
// Espacio con orientación (UCS, A*): estado = celda·4 + rumbo.
struct Space {
  const Grid &g;
  bool oriented;
  float turnCost;

  Cell cellOf(uint8_t s) const { return cellFromIndex(oriented ? s / 4 : s); }
  uint8_t headingOf(uint8_t s) const { return oriented ? (uint8_t)(s & 3) : (uint8_t)ANY; }
  uint8_t make(Cell k, uint8_t h) const {
    return (uint8_t)(oriented ? cellIndex(k) * 4 + h : cellIndex(k));
  }

  // Sucesores en orden fijo. Celdas: N, E, S, O. Con orientación: avanzar, izquierda, derecha.
  int successors(uint8_t s, Successor out[4]) const {
    int n = 0;
    Cell k = cellOf(s);
    if (!oriented) {
      for (uint8_t h = 0; h < 4; h++) {
        Cell nk = step(k, h);
        if (g.isFree(nk)) out[n++] = {make(nk, 0), 1.0f};
      }
    } else {
      uint8_t h = headingOf(s);
      Cell nk = step(k, h);
      if (g.isFree(nk)) out[n++] = {make(nk, h), 1.0f};
      out[n++] = {make(k, turnLeft(h)), turnCost};
      out[n++] = {make(k, turnRight(h)), turnCost};
    }
    return n;
  }
};

template <class Frontier>
SearchResult runSearch(const Grid &g, Cell start, uint8_t startHeading, Cell goal,
                       const SearchParams &p, Frontier &open) {
  SearchResult r;
  const bool oriented = (p.algo == Algorithm::UCS || p.algo == Algorithm::ASTAR);
  Space sp{g, oriented, p.turnCost};

  bool closed[NSTATES];
  bool discovered[NSTATES];
  float bestG[NSTATES];
  uint8_t parent[NSTATES];
  for (int i = 0; i < NSTATES; i++) {
    closed[i] = false;
    discovered[i] = false;
    bestG[i] = INF_COST;
    parent[i] = NONE;
  }

  uint16_t seq = 0;
  auto pushStart = [&](uint8_t h) {
    uint8_t s = sp.make(start, h);
    if (discovered[s]) return;
    discovered[s] = true;
    bestG[s] = 0;
    open.push({s, NONE, 0.0f, heuristic(start, h, goal, p), seq++});
  };
  if (oriented && startHeading == ANY)
    for (uint8_t h = 0; h < 4; h++) pushStart(h);
  else
    pushStart(oriented ? startHeading : 0);

  uint8_t goalState = NONE;
  while (!open.empty()) {
    Entry e = open.pop();  // FIFO, LIFO o menor prioridad
    if (closed[e.state]) continue;
    closed[e.state] = true;
    parent[e.state] = e.parent;
    bestG[e.state] = e.g;
    r.expanded.push_back((uint8_t)cellIndex(sp.cellOf(e.state)));
    if (sp.cellOf(e.state) == goal) {
      goalState = e.state;
      break;
    }
    Successor succ[4];
    int ns = sp.successors(e.state, succ);
    // DFS: se apilan al revés para que el primero en expandirse sea N (luego E, S, O).
    for (int k = 0; k < ns; k++) {
      const Successor &su = (p.algo == Algorithm::DFS) ? succ[ns - 1 - k] : succ[k];
      uint8_t n = su.state;
      if (closed[n]) continue;
      float gn = e.g + su.cost;
      bool push = false;
      switch (p.algo) {
        case Algorithm::BFS: push = !discovered[n]; break;  // se marca al descubrir
        case Algorithm::DFS: push = true; break;            // la pila decide
        default: push = gn < bestG[n]; break;                // relajación
      }
      if (!push) continue;
      discovered[n] = true;
      if (oriented) bestG[n] = gn;
      Cell nk = sp.cellOf(n);
      open.push({n, e.state, gn, gn + heuristic(nk, sp.headingOf(n), goal, p), seq++});
    }
  }

  if (goalState == NONE) return r;

  // Reconstrucción
  std::vector<Cell> rev;
  for (uint8_t s = goalState; s != NONE; s = parent[s]) {
    Cell k = sp.cellOf(s);
    if (rev.empty() || rev.back() != k) rev.push_back(k);  // los giros no cambian de celda
  }
  r.path.assign(rev.rbegin(), rev.rend());
  r.found = true;
  r.cost = evaluatePath(r.path, startHeading, p.turnCost, &r.turns, &r.endHeading);
  r.cells = (uint16_t)(r.path.size() - 1);
  return r;
}

}  // namespace

SearchResult search(const Grid &g, Cell start, uint8_t startHeading, Cell goal,
                    const SearchParams &p) {
  uint32_t t0 = nowMicros();
  SearchResult r;
  if (!inBounds(start) || !inBounds(goal) || g.isBlocked(goal) || g.isBlocked(start)) {
    r.timeUs = nowMicros() - t0;
    return r;
  }
  // Las fronteras son grandes (≈16 KB): estáticas para no agotar la pila de la tarea.
  // El planificador corre en una sola tarea, así que no hay acceso concurrente.
  switch (p.algo) {
    case Algorithm::BFS: { static FifoFrontier f; f.reset(); r = runSearch(g, start, startHeading, goal, p, f); break; }
    case Algorithm::DFS: { static LifoFrontier f; f.reset(); r = runSearch(g, start, startHeading, goal, p, f); break; }
    default: { static PriorityFrontier f; f.reset(); r = runSearch(g, start, startHeading, goal, p, f); break; }
  }
  r.timeUs = nowMicros() - t0;
  return r;
}

}  // namespace planner
