#pragma once
// Cuadrícula de 6×6 celdas. No depende de Arduino: se prueba en la PC.
//
// Convención de ejes (ver README):
//   celda (f, c): f = fila (0..5), c = columna (0..5)
//   centro de la celda: x = 0.25 + 0.5·c m, y = 0.25 + 0.5·f m
//   N = +y (f+1), E = +x (c+1), S = −y (f−1), O = −x (c−1)
//   rumbo θ: E = 0°, N = +90°, O = 180°, S = −90° (positivo = antihorario)
#include <stdint.h>

namespace planner {

constexpr int ROWS = 6;
constexpr int COLS = 6;
constexpr int NCELLS = ROWS * COLS;  // 36
constexpr int NHEAD = 4;
constexpr int NSTATES = NCELLS * NHEAD;  // 144
constexpr int MAX_GOALS = 12;
constexpr float INF_COST = 1e9f;

// Orden horario: N, E, S, O. Girar a la derecha = +1, a la izquierda = +3 (mod 4).
enum Heading : uint8_t { N = 0, E = 1, S = 2, O = 3, ANY = 255 };

constexpr int8_t DF[4] = {+1, 0, -1, 0};
constexpr int8_t DC[4] = {0, +1, 0, -1};

struct Cell {
  int8_t f = 0;
  int8_t c = 0;
  Cell() = default;
  Cell(int ff, int cc) : f((int8_t)ff), c((int8_t)cc) {}
  bool operator==(const Cell &o) const { return f == o.f && c == o.c; }
  bool operator!=(const Cell &o) const { return !(*this == o); }
};

inline int cellIndex(Cell k) { return k.f * COLS + k.c; }
inline Cell cellFromIndex(int i) { return Cell(i / COLS, i % COLS); }
inline bool inBounds(Cell k) { return k.f >= 0 && k.f < ROWS && k.c >= 0 && k.c < COLS; }
inline Cell step(Cell k, uint8_t h) { return Cell(k.f + DF[h], k.c + DC[h]); }
inline uint8_t turnRight(uint8_t h) { return (uint8_t)((h + 1) & 3); }
inline uint8_t turnLeft(uint8_t h) { return (uint8_t)((h + 3) & 3); }

// Número de giros de 90° para pasar del rumbo h al d (0, 1 o 2).
inline uint8_t turnsBetween(uint8_t h, uint8_t d) {
  if (h == ANY) return 0;
  uint8_t diff = (uint8_t)((d - h + 4) & 3);
  return diff == 2 ? 2 : (diff == 0 ? 0 : 1);
}

// Giro con signo en grados: +90 = izquierda (antihorario), −90 = derecha, 180.
inline int16_t turnDegrees(uint8_t h, uint8_t d) {
  uint8_t diff = (uint8_t)((d - h + 4) & 3);
  if (diff == 0) return 0;
  if (diff == 1) return -90;
  if (diff == 3) return +90;
  return 180;
}

// Dirección del movimiento de a hacia b (vecinos). 255 si no son vecinos.
inline uint8_t directionBetween(Cell a, Cell b) {
  for (uint8_t h = 0; h < 4; h++)
    if (step(a, h) == b) return h;
  return 255;
}

// Rumbo en grados del robot para cada Heading (E = 0, N = 90, S = −90, O = 180).
inline float headingDegrees(uint8_t h) {
  switch (h) {
    case N: return 90.0f;
    case E: return 0.0f;
    case S: return -90.0f;
    default: return 180.0f;
  }
}

inline char headingChar(uint8_t h) {
  static const char k[] = {'N', 'E', 'S', 'O'};
  return h < 4 ? k[h] : '?';
}

inline bool parseHeading(char ch, uint8_t &h) {
  switch (ch) {
    case 'N': case 'n': h = N; return true;
    case 'E': case 'e': h = E; return true;
    case 'S': case 's': h = S; return true;
    case 'O': case 'o': case 'W': case 'w': h = O; return true;
    default: return false;
  }
}

class Grid {
 public:
  Grid() { clear(); }
  void clear() {
    for (int i = 0; i < NCELLS; i++) blocked_[i] = false;
  }
  void setBlocked(Cell k, bool b) {
    if (inBounds(k)) blocked_[cellIndex(k)] = b;
  }
  bool isBlocked(Cell k) const { return !inBounds(k) || blocked_[cellIndex(k)]; }
  bool isFree(Cell k) const { return !isBlocked(k); }

 private:
  bool blocked_[NCELLS];
};

}  // namespace planner
