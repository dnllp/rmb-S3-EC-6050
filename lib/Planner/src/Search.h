#pragma once
// BFS, DFS, UCS y A* con un solo motor de búsqueda; solo cambia la frontera.
#include <vector>
#include "Grid.h"

namespace planner {

enum class Algorithm : uint8_t { BFS = 0, DFS = 1, UCS = 2, ASTAR = 3 };

// MANHATTAN: |Δf| + |Δc|.
// MANHATTAN_TURNS: Manhattan + c_g·τ, τ = giros de 90° inevitables (admisible).
enum class Heuristic : uint8_t { MANHATTAN = 0, MANHATTAN_TURNS = 1 };

struct SearchParams {
  Algorithm algo = Algorithm::ASTAR;
  float turnCost = 0.5f;  // c_g: costo de girar 90°
  Heuristic heuristic = Heuristic::MANHATTAN;
};

struct SearchResult {
  bool found = false;
  std::vector<Cell> path;         // incluye inicio y meta
  float cost = 0;                 // celdas + c_g·giros (métrica común)
  uint16_t cells = 0;             // celdas avanzadas
  uint16_t turns = 0;             // giros de 90° (180° cuenta 2)
  uint8_t endHeading = ANY;       // rumbo al llegar
  std::vector<uint8_t> expanded;  // índice de celda de cada nodo expandido, en orden
  uint32_t timeUs = 0;
};

// startHeading puede ser ANY (sin orientación inicial; se usa en la matriz de costos).
SearchResult search(const Grid &g, Cell start, uint8_t startHeading, Cell goal,
                    const SearchParams &p);

// Evalúa una ruta con la métrica común: celdas + c_g · giros de 90°.
float evaluatePath(const std::vector<Cell> &path, uint8_t startHeading, float turnCost,
                   uint16_t *turnsOut = nullptr, uint8_t *endHeadingOut = nullptr);

// Heurística de A* (expuesta para pruebas).
float heuristic(Cell k, uint8_t h, Cell goal, const SearchParams &p);
uint8_t unavoidableTurns(Cell k, uint8_t h, Cell goal);

const char *algorithmName(Algorithm a);         // "bfs", "dfs", "ucs", "astar"
bool parseAlgorithm(const char *s, Algorithm &a);

uint32_t nowMicros();

}  // namespace planner
