#include "TourOptimizer.h"

#include <algorithm>

namespace planner {

float tourCost(const CostMatrix &m, const std::vector<uint8_t> &order, bool returnHome) {
  float c = 0;
  uint8_t prev = 0;
  for (uint8_t j : order) {
    c += m[prev][j];
    prev = j;
  }
  if (returnHome && !order.empty()) c += m[prev][0];
  return c;
}

std::vector<uint8_t> heldKarp(const CostMatrix &m, bool returnHome, float *costOut) {
  const int n = (int)m.size() - 1;  // número de metas
  std::vector<uint8_t> order;
  if (n <= 0) {
    if (costOut) *costOut = 0;
    return order;
  }
  const uint32_t full = (1u << n) - 1;
  // dp[mask·n + j]: costo mínimo desde el inicio, visitando 'mask', terminando en la meta j.
  // Con n = 10: 1024 × 10 × 4 B = 40 KB (+10 KB de padres).
  std::vector<float> dp((size_t)(full + 1) * n, INF_COST);
  std::vector<uint8_t> par((size_t)(full + 1) * n, 255);
  for (int j = 0; j < n; j++) dp[(size_t)(1u << j) * n + j] = m[0][j + 1];
  for (uint32_t mask = 1; mask <= full; mask++) {
    for (int j = 0; j < n; j++) {
      if (!(mask & (1u << j))) continue;
      float cur = dp[(size_t)mask * n + j];
      if (cur >= INF_COST) continue;
      for (int k = 0; k < n; k++) {
        if (mask & (1u << k)) continue;
        uint32_t nm = mask | (1u << k);
        float c = cur + m[j + 1][k + 1];
        float &slot = dp[(size_t)nm * n + k];
        if (c < slot) {
          slot = c;
          par[(size_t)nm * n + k] = (uint8_t)j;
        }
      }
    }
  }
  float best = INF_COST;
  int last = -1;
  for (int j = 0; j < n; j++) {
    float c = dp[(size_t)full * n + j] + (returnHome ? m[j + 1][0] : 0.0f);
    if (c < best) {
      best = c;
      last = j;
    }
  }
  if (last < 0) {
    if (costOut) *costOut = INF_COST;
    return order;
  }
  uint32_t mask = full;
  int j = last;
  while (j >= 0 && j != 255) {
    order.push_back((uint8_t)(j + 1));
    uint8_t pj = par[(size_t)mask * n + j];
    mask &= ~(1u << j);
    j = (pj == 255) ? -1 : pj;
  }
  std::reverse(order.begin(), order.end());
  if (costOut) *costOut = best;
  return order;
}

std::vector<uint8_t> nearestNeighbor2Opt(const CostMatrix &m, bool returnHome, float *costOut) {
  const int n = (int)m.size() - 1;
  std::vector<uint8_t> order;
  std::vector<bool> used(n + 1, false);
  uint8_t cur = 0;
  for (int step = 0; step < n; step++) {
    float best = INF_COST * 2;
    int bj = -1;
    for (int j = 1; j <= n; j++) {
      if (!used[j] && m[cur][j] < best) {
        best = m[cur][j];
        bj = j;
      }
    }
    used[bj] = true;
    order.push_back((uint8_t)bj);
    cur = (uint8_t)bj;
  }
  // 2-opt: invertir tramos mientras mejore el costo total (la matriz puede ser asimétrica).
  float best = tourCost(m, order, returnHome);
  bool improved = true;
  while (improved) {
    improved = false;
    for (int i = 0; i < n - 1; i++) {
      for (int k = i + 1; k < n; k++) {
        std::reverse(order.begin() + i, order.begin() + k + 1);
        float c = tourCost(m, order, returnHome);
        if (c + 1e-6f < best) {
          best = c;
          improved = true;
        } else {
          std::reverse(order.begin() + i, order.begin() + k + 1);
        }
      }
    }
  }
  if (costOut) *costOut = best;
  return order;
}

TourResult planTour(const Grid &g, Cell start, uint8_t startHeading,
                    const std::vector<Cell> &goals, const TourParams &p) {
  uint32_t t0 = nowMicros();
  TourResult t;
  if (!inBounds(start) || g.isBlocked(start)) {
    t.error = "inicio invalido u ocupado";
    return t;
  }
  if (goals.empty()) {
    t.error = "no hay metas";
    return t;
  }
  if (goals.size() > (size_t)MAX_GOALS) {
    t.error = "mas de 12 metas";
    return t;
  }

  // 1) Metas alcanzables desde el inicio
  std::vector<uint8_t> reach;  // índices en 'goals'
  std::vector<float> fromStart;
  for (size_t i = 0; i < goals.size(); i++) {
    SearchResult r = search(g, start, startHeading, goals[i], p.search);
    t.expandedMatrix += (uint32_t)r.expanded.size();
    if (r.found) {
      reach.push_back((uint8_t)i);
      fromStart.push_back(r.cost);
    } else {
      t.unreachable.push_back((uint8_t)i);
    }
  }
  if (reach.empty()) {
    t.error = "ninguna meta es alcanzable";
    t.timeUs = nowMicros() - t0;
    return t;
  }

  // 2) Matriz de costos. Entre metas no se conoce el rumbo de llegada: se busca con
  //    rumbo libre (ANY). El tramo final se vuelve a planear con el rumbo real.
  const int n = (int)reach.size();
  CostMatrix m(n + 1, std::vector<float>(n + 1, 0.0f));
  for (int j = 0; j < n; j++) m[0][j + 1] = fromStart[j];
  for (int i = 0; i < n; i++) {
    for (int j = 0; j < n; j++) {
      if (i == j) continue;
      SearchResult r = search(g, goals[reach[i]], ANY, goals[reach[j]], p.search);
      t.expandedMatrix += (uint32_t)r.expanded.size();
      m[i + 1][j + 1] = r.found ? r.cost : INF_COST;
    }
    if (p.returnHome) {
      SearchResult r = search(g, goals[reach[i]], ANY, start, p.search);
      t.expandedMatrix += (uint32_t)r.expanded.size();
      m[i + 1][0] = r.found ? r.cost : INF_COST;
    }
  }

  // 3) Orden de visita
  std::vector<uint8_t> ord;
  if (n <= HELD_KARP_MAX) {
    ord = heldKarp(m, p.returnHome);
    t.exact = true;
  } else {
    ord = nearestNeighbor2Opt(m, p.returnHome);
  }
  for (uint8_t k : ord) t.order.push_back(reach[k - 1]);

  // 4) Tramo por tramo con el rumbo real de llegada
  Cell cur = start;
  uint8_t curH = startHeading;
  auto addLeg = [&](uint8_t goalIndex, Cell target) -> bool {
    Leg leg;
    leg.goalIndex = goalIndex;
    leg.res = search(g, cur, curH, target, p.search);
    if (!leg.res.found) return false;
    t.expanded += (uint32_t)leg.res.expanded.size();
    t.cost += leg.res.cost;
    t.cells += leg.res.cells;
    t.turns += leg.res.turns;
    cur = target;
    curH = leg.res.endHeading;
    t.legs.push_back(std::move(leg));
    return true;
  };
  for (uint8_t gi : t.order) {
    if (!addLeg(gi, goals[gi])) {
      t.error = "fallo un tramo";
      t.timeUs = nowMicros() - t0;
      return t;
    }
  }
  if (p.returnHome && !addLeg(LEG_RETURN_HOME, start)) {
    t.error = "no se puede regresar al inicio";
    t.timeUs = nowMicros() - t0;
    return t;
  }
  t.ok = true;
  t.timeUs = nowMicros() - t0;
  return t;
}

}  // namespace planner
