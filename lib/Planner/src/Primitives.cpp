#include "Primitives.h"

#include <stddef.h>

namespace planner {

void appendPathPrimitives(const std::vector<Cell> &path, uint8_t startHeading,
                          std::vector<Primitive> &out) {
  uint8_t h = startHeading;
  size_t i = 1;
  while (i < path.size()) {
    uint8_t d = directionBetween(path[i - 1], path[i]);
    if (d == 255) return;  // ruta inválida: no debería pasar
    if (h != ANY && d != h) {
      Primitive t;
      t.type = PrimType::TURN;
      t.turnDeg = turnDegrees(h, d);
      t.heading = d;
      t.target = path[i - 1];
      out.push_back(t);
    }
    h = d;
    // Agrupar celdas consecutivas en la misma dirección
    size_t j = i;
    while (j + 1 < path.size() && directionBetween(path[j], path[j + 1]) == d) j++;
    Primitive f;
    f.type = PrimType::FORWARD;
    f.cells = (uint8_t)(j - i + 1);
    f.heading = d;
    f.target = path[j];
    out.push_back(f);
    i = j + 1;
  }
}

std::vector<Primitive> tourToPrimitives(const TourResult &t, Cell start, uint8_t startHeading) {
  std::vector<Primitive> out;
  uint8_t h = startHeading;
  (void)start;
  for (const Leg &leg : t.legs) {
    size_t before = out.size();
    appendPathPrimitives(leg.res.path, h, out);
    if (out.size() > before) out.back().goalReached = leg.goalIndex;
    if (leg.res.endHeading != ANY) h = leg.res.endHeading;
  }
  return out;
}

}  // namespace planner
