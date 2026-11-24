#pragma once
// Orden de visita de las metas: Held–Karp exacto hasta 10 metas,
// vecino más cercano + 2-opt para 11 y 12.
#include <vector>
#include "Search.h"

namespace planner {

constexpr int HELD_KARP_MAX = 10;
constexpr uint8_t LEG_RETURN_HOME = 254;

struct TourParams {
  SearchParams search;
  bool returnHome = false;
};

struct Leg {
  uint8_t goalIndex = 0;  // índice en la lista de metas original; LEG_RETURN_HOME = regreso
  SearchResult res;
};

struct TourResult {
  bool ok = false;
  const char *error = nullptr;
  std::vector<uint8_t> order;        // índices de metas en orden de visita
  std::vector<uint8_t> unreachable;  // metas que no se pueden alcanzar
  std::vector<Leg> legs;             // un tramo por meta (+ regreso)
  bool exact = false;                // true si el orden salió de Held–Karp
  float cost = 0;
  uint16_t cells = 0;
  uint16_t turns = 0;
  uint32_t expanded = 0;        // nodos expandidos en los tramos finales
  uint32_t expandedMatrix = 0;  // nodos expandidos al llenar la matriz de costos
  uint32_t timeUs = 0;
};

using CostMatrix = std::vector<std::vector<float>>;

// Nodo 0 = inicio, nodos 1..n = metas. Devuelve el orden de metas (1..n).
std::vector<uint8_t> heldKarp(const CostMatrix &m, bool returnHome, float *costOut = nullptr);
std::vector<uint8_t> nearestNeighbor2Opt(const CostMatrix &m, bool returnHome,
                                         float *costOut = nullptr);
float tourCost(const CostMatrix &m, const std::vector<uint8_t> &order, bool returnHome);

TourResult planTour(const Grid &g, Cell start, uint8_t startHeading,
                    const std::vector<Cell> &goals, const TourParams &p);

}  // namespace planner
