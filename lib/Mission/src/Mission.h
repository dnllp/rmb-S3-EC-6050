#pragma once
// Máquina de estados de la misión y cola de primitivas.
// INACTIVO → PLANEANDO → LISTO → EJECUTANDO ⇄ PAUSADO → COMPLETADO;  FALLA desde cualquier estado.
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <vector>
#include "Planner.h"
#include "Motion.h"
#include "Pose.h"

class Mission {
 public:
  enum class State : uint8_t { INACTIVO, PLANEANDO, LISTO, EJECUTANDO, PAUSADO, COMPLETADO, FALLA };

  struct PlanRequest {
    planner::Cell start;
    uint8_t heading = planner::E;
    std::vector<planner::Cell> goals;
    planner::Algorithm algo = planner::Algorithm::ASTAR;
    planner::Heuristic heuristic = planner::Heuristic::MANHATTAN;
    bool returnHome = false;
  };

  // Paso de ejecución: una primitiva ya traducida a coordenadas del piso
  struct Step {
    enum Kind : uint8_t { FORWARD, TURN, PWM } kind = FORWARD;
    float tx = 0, ty = 0, th = 0;  // FORWARD: punto y rumbo; TURN: rumbo
    float dutyL = 0, dutyR = 0;
    uint32_t ms = 0;
    planner::Cell target;
    uint8_t goalReached = planner::NO_GOAL;
    char label[24] = "";
  };

  void begin(Motion *m, Pose *p);

  // --- mapa (persistente en NVS)
  void setObstacles(const std::vector<planner::Cell> &obs);
  planner::Grid grid() const;

  // --- planificación (la calcula la tarea planner en el núcleo 0)
  bool requestPlan(const PlanRequest &r, const char **err);
  void plannerLoop();  // cuerpo de la tarea planner (no regresa)
  void onPlanReady(void (*cb)()) { onPlanReady_ = cb; }

  // --- ejecución (tarea mission, núcleo 1, cada 50 ms)
  void step();
  bool execute(bool confirmUnreachable, const char **err);
  bool togglePause(const char **err);
  void emergencyStop();                    // freno + EN en bajo; → INACTIVO
  void fail(const char *msg);              // → FALLA
  bool runTest(const Step &s, const char **err);  // una primitiva suelta (calibración)
  bool testForward(int cells, const char **err);
  bool testTurn(int degrees, const char **err);
  bool testPwm(float l, float r, uint32_t ms, const char **err);
  void setPoseAt(planner::Cell c, uint8_t heading);  // colocar el robot a mano

  // --- consulta (copias seguras para otras tareas)
  State state() const { return state_; }
  static const char *stateName(State s);
  const char *message() const { return message_; }
  size_t stepIndex() const { return idx_; }
  size_t stepCount() const;
  void currentLabel(char *out, size_t n) const;
  planner::Cell currentCell() const;
  uint32_t goalsReached() const { return goalsReached_; }

  // Acceso al último plan bajo candado: f(req, tour) se ejecuta con el mutex tomado
  template <class F>
  void withPlan(F f) const {
    xSemaphoreTake(lock_, portMAX_DELAY);
    f(planValid_, lastReq_, lastTour_);
    xSemaphoreGive(lock_);
  }

 private:
  void startStep();
  void finishStep();
  float continuousHeading(uint8_t h) const;
  float snapHeading(float th) const;
  void buildSteps();
  void saveObstacles();
  void loadObstacles();

  Motion *motion_ = nullptr;
  Pose *pose_ = nullptr;
  SemaphoreHandle_t lock_ = nullptr;
  TaskHandle_t plannerTask_ = nullptr;

  volatile State state_ = State::INACTIVO;
  const char *message_ = "";
  planner::Grid grid_;

  PlanRequest pendingReq_;
  PlanRequest lastReq_;
  planner::TourResult lastTour_;
  bool planValid_ = false;
  std::vector<Step> steps_;
  volatile size_t idx_ = 0;
  bool stepStarted_ = false;
  bool pauseRequested_ = false;
  uint32_t pauseT0_ = 0;
  bool inPause_ = false;
  bool testMode_ = false;
  uint32_t goalsReached_ = 0;
  bool poseValid_ = false;
  planner::Cell lastCell_;
  uint8_t lastHeading_ = planner::E;
  void (*onPlanReady_)() = nullptr;
};
