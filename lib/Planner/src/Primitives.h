#pragma once
// Traduce rutas de celdas a primitivas de movimiento: AVANZAR(n) y GIRAR(±90°/180°).
// E, E, E, N  →  AVANZAR(3), GIRAR(+90°), AVANZAR(1)
#include <vector>
#include "TourOptimizer.h"

namespace planner {

enum class PrimType : uint8_t { FORWARD = 0, TURN = 1 };

constexpr uint8_t NO_GOAL = 255;

struct Primitive {
  PrimType type = PrimType::FORWARD;
  uint8_t cells = 0;        // FORWARD: celdas a avanzar
  int16_t turnDeg = 0;      // TURN: +90 izquierda, −90 derecha, 180
  uint8_t heading = E;      // rumbo absoluto al terminar la primitiva
  Cell target;              // celda donde termina la primitiva
  uint8_t goalReached = NO_GOAL;  // meta alcanzada al terminar (índice original o LEG_RETURN_HOME)
};

void appendPathPrimitives(const std::vector<Cell> &path, uint8_t startHeading,
                          std::vector<Primitive> &out);

// Todas las primitivas de un plan, tramo por tramo (el robot se detiene en cada meta).
std::vector<Primitive> tourToPrimitives(const TourResult &t, Cell start, uint8_t startHeading);

}  // namespace planner
