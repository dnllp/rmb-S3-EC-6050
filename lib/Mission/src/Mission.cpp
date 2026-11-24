#include "Mission.h"

#include <Preferences.h>
#include <math.h>

#include "roomba_config.h"
#include "roomba_settings.h"

using namespace planner;

namespace {
constexpr float TWO_PI_F = 2.0f * (float)M_PI;
inline float cellCenterX(Cell c) { return 0.25f + CELL_SIZE_M * c.c; }
inline float cellCenterY(Cell c) { return 0.25f + CELL_SIZE_M * c.f; }
}  // namespace

const char *Mission::stateName(State s) {
  switch (s) {
    case State::INACTIVO: return "INACTIVO";
    case State::PLANEANDO: return "PLANEANDO";
    case State::LISTO: return "LISTO";
    case State::EJECUTANDO: return "EJECUTANDO";
    case State::PAUSADO: return "PAUSADO";
    case State::COMPLETADO: return "COMPLETADO";
    case State::FALLA: return "FALLA";
  }
  return "?";
}

void Mission::begin(Motion *m, Pose *p) {
  motion_ = m;
  pose_ = p;
  lock_ = xSemaphoreCreateMutex();
  loadObstacles();
}

// ---------------------------------------------------------------- mapa
void Mission::setObstacles(const std::vector<Cell> &obs) {
  xSemaphoreTake(lock_, portMAX_DELAY);
  grid_.clear();
  for (const Cell &c : obs) grid_.setBlocked(c, true);
  planValid_ = false;  // un plan viejo ya no vale con otro mapa
  if (state_ == State::LISTO) state_ = State::INACTIVO;
  xSemaphoreGive(lock_);
  saveObstacles();
}

Grid Mission::grid() const {
  xSemaphoreTake(lock_, portMAX_DELAY);
  Grid g = grid_;
  xSemaphoreGive(lock_);
  return g;
}

void Mission::saveObstacles() {
  uint8_t bits[NCELLS];
  Grid g = grid();
  for (int i = 0; i < NCELLS; i++) bits[i] = g.isBlocked(cellFromIndex(i)) ? 1 : 0;
  Preferences p;
  if (!p.begin("mapa", false)) return;
  p.putBytes("obs", bits, sizeof(bits));
  p.end();
}

void Mission::loadObstacles() {
  uint8_t bits[NCELLS];
  Preferences p;
  if (!p.begin("mapa", true)) return;
  size_t n = p.getBytes("obs", bits, sizeof(bits));
  p.end();
  if (n != sizeof(bits)) return;
  for (int i = 0; i < NCELLS; i++) grid_.setBlocked(cellFromIndex(i), bits[i] != 0);
}

// ---------------------------------------------------------------- planificación
bool Mission::requestPlan(const PlanRequest &r, const char **err) {
  State s = state_;
  if (s == State::EJECUTANDO || s == State::PAUSADO) {
    *err = "hay una mision en curso";
    return false;
  }
  if (s == State::PLANEANDO) {
    *err = "ya se esta planeando";
    return false;
  }
  if (!plannerTask_) {
    *err = "el planificador aun no arranca";
    return false;
  }
  if (r.goals.empty() || r.goals.size() > (size_t)MAX_GOALS) {
    *err = "se necesitan de 1 a 12 metas";
    return false;
  }
  xSemaphoreTake(lock_, portMAX_DELAY);
  pendingReq_ = r;
  state_ = State::PLANEANDO;
  message_ = "planeando";
  xSemaphoreGive(lock_);
  if (motion_->faulted()) motion_->clearFault();
  xTaskNotifyGive(plannerTask_);
  return true;
}

void Mission::plannerLoop() {
  plannerTask_ = xTaskGetCurrentTaskHandle();
  for (;;) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    PlanRequest req;
    Grid g;
    xSemaphoreTake(lock_, portMAX_DELAY);
    req = pendingReq_;
    g = grid_;
    xSemaphoreGive(lock_);

    TourParams tp;
    tp.search.algo = req.algo;
    tp.search.heuristic = req.heuristic;
    tp.search.turnCost = gSettings.turnCost;
    tp.returnHome = req.returnHome;
    TourResult t = planTour(g, req.start, req.heading, req.goals, tp);

    xSemaphoreTake(lock_, portMAX_DELAY);
    lastReq_ = req;
    lastTour_ = std::move(t);
    planValid_ = lastTour_.ok;
    if (planValid_) {
      buildSteps();
      state_ = State::LISTO;
      message_ = lastTour_.unreachable.empty() ? "plan listo" : "plan listo; hay metas inalcanzables";
    } else {
      steps_.clear();
      state_ = State::INACTIVO;
      message_ = lastTour_.error ? lastTour_.error : "no se pudo planear";
    }
    xSemaphoreGive(lock_);
    if (onPlanReady_) onPlanReady_();
  }
}

// Se llama con el mutex tomado
void Mission::buildSteps() {
  steps_.clear();
  std::vector<Primitive> prims = tourToPrimitives(lastTour_, lastReq_.start, lastReq_.heading);
  for (const Primitive &p : prims) {
    Step s;
    s.target = p.target;
    s.goalReached = p.goalReached;
    if (p.type == PrimType::FORWARD) {
      s.kind = Step::FORWARD;
      s.tx = cellCenterX(p.target);
      s.ty = cellCenterY(p.target);
      s.th = headingDegrees(p.heading) * DEG_TO_RAD;  // se ajusta al rumbo continuo al iniciar
      snprintf(s.label, sizeof(s.label), "AVANZAR(%u)", p.cells);
    } else {
      s.kind = Step::TURN;
      s.th = headingDegrees(p.heading) * DEG_TO_RAD;
      snprintf(s.label, sizeof(s.label), "GIRAR(%+d)", p.turnDeg);
    }
    steps_.push_back(s);
  }
}

// ---------------------------------------------------------------- ejecución
float Mission::continuousHeading(uint8_t h) const {
  // Rumbo absoluto (múltiplo de 90°) en la "vuelta" más cercana al rumbo continuo actual
  float a = headingDegrees(h) * DEG_TO_RAD;
  float k = roundf((pose_->theta - a) / TWO_PI_F);
  return a + k * TWO_PI_F;
}

float Mission::snapHeading(float th) const {
  float q = (float)M_PI / 2.0f;
  return roundf(th / q) * q;
}

bool Mission::execute(bool confirmUnreachable, const char **err) {
  xSemaphoreTake(lock_, portMAX_DELAY);
  bool ok = state_ == State::LISTO && planValid_;
  bool needConfirm = ok && !lastTour_.unreachable.empty() && !confirmUnreachable;
  Cell start = lastReq_.start;
  uint8_t h = lastReq_.heading;
  xSemaphoreGive(lock_);
  if (!ok) {
    *err = "no hay un plan listo";
    return false;
  }
  if (needConfirm) {
    *err = "hay metas inalcanzables: confirma para continuar sin ellas";
    return false;
  }
  // La pose arranca en el centro de la celda de inicio, salvo que el robot ya esté ahí
  // con esa orientación al terminar la misión anterior.
  if (!(poseValid_ && lastCell_ == start && lastHeading_ == h)) setPoseAt(start, h);
  idx_ = 0;
  stepStarted_ = false;
  pauseRequested_ = false;
  inPause_ = false;
  testMode_ = false;
  goalsReached_ = 0;
  message_ = "ejecutando";
  state_ = State::EJECUTANDO;
  return true;
}

void Mission::setPoseAt(Cell c, uint8_t heading) {
  float th = headingDegrees(heading) * DEG_TO_RAD;
  pose_->reset(cellCenterX(c), cellCenterY(c), th);
  poseValid_ = true;
  lastCell_ = c;
  lastHeading_ = heading;
}

bool Mission::togglePause(const char **err) {
  if (state_ == State::EJECUTANDO) {
    pauseRequested_ = true;  // se detiene al terminar la primitiva en curso
    message_ = "pausa al terminar la primitiva";
    return true;
  }
  if (state_ == State::PAUSADO) {
    pauseRequested_ = false;
    message_ = "ejecutando";
    state_ = State::EJECUTANDO;
    return true;
  }
  *err = "no hay mision en curso";
  return false;
}

void Mission::emergencyStop() {
  motion_->emergency();
  pauseRequested_ = false;
  poseValid_ = false;  // ya no se sabe dónde quedó exactamente
  testMode_ = false;
  message_ = "paro";
  state_ = State::INACTIVO;
}

void Mission::fail(const char *msg) {
  motion_->release();  // freno y EN en bajo
  poseValid_ = false;
  testMode_ = false;
  message_ = msg;
  state_ = State::FALLA;
}

bool Mission::runTest(const Step &s, const char **err) {
  State st = state_;
  if (st == State::EJECUTANDO || st == State::PAUSADO || st == State::PLANEANDO) {
    *err = "hay una mision en curso";
    return false;
  }
  if (motion_->faulted()) motion_->clearFault();
  xSemaphoreTake(lock_, portMAX_DELAY);
  steps_.clear();
  steps_.push_back(s);
  planValid_ = false;  // la prueba reemplaza la cola: hay que volver a planear
  xSemaphoreGive(lock_);
  idx_ = 0;
  stepStarted_ = false;
  pauseRequested_ = false;
  inPause_ = false;
  testMode_ = true;
  message_ = "prueba";
  state_ = State::EJECUTANDO;
  return true;
}

bool Mission::testForward(int cells, const char **err) {
  if (cells < 1 || cells > 5) {
    *err = "celdas de 1 a 5";
    return false;
  }
  Step s;
  s.kind = Step::FORWARD;
  s.th = snapHeading(pose_->theta);
  s.tx = pose_->x + cells * CELL_SIZE_M * cosf(s.th);
  s.ty = pose_->y + cells * CELL_SIZE_M * sinf(s.th);
  snprintf(s.label, sizeof(s.label), "AVANZAR(%d)", cells);
  return runTest(s, err);
}

bool Mission::testTurn(int degrees, const char **err) {
  if (degrees != 90 && degrees != -90 && degrees != 180) {
    *err = "grados: 90, -90 o 180";
    return false;
  }
  Step s;
  s.kind = Step::TURN;
  s.th = snapHeading(pose_->theta) + degrees * DEG_TO_RAD;
  snprintf(s.label, sizeof(s.label), "GIRAR(%+d)", degrees);
  return runTest(s, err);
}

bool Mission::testPwm(float l, float r, uint32_t ms, const char **err) {
  if (ms > 5000) ms = 5000;
  Step s;
  s.kind = Step::PWM;
  s.dutyL = l;
  s.dutyR = r;
  s.ms = ms;
  snprintf(s.label, sizeof(s.label), "PWM(%.2f,%.2f)", l, r);
  return runTest(s, err);
}

size_t Mission::stepCount() const {
  xSemaphoreTake(lock_, portMAX_DELAY);
  size_t n = steps_.size();
  xSemaphoreGive(lock_);
  return n;
}

void Mission::currentLabel(char *out, size_t n) const {
  xSemaphoreTake(lock_, portMAX_DELAY);
  if (idx_ < steps_.size() && (state_ == State::EJECUTANDO || state_ == State::PAUSADO))
    snprintf(out, n, "%s", steps_[idx_].label);
  else
    snprintf(out, n, "-");
  xSemaphoreGive(lock_);
}

Cell Mission::currentCell() const {
  int c = (int)floorf(pose_->x / CELL_SIZE_M);
  int f = (int)floorf(pose_->y / CELL_SIZE_M);
  return Cell(f, c);
}

void Mission::startStep() {
  Step s;
  xSemaphoreTake(lock_, portMAX_DELAY);
  s = steps_[idx_];
  xSemaphoreGive(lock_);
  switch (s.kind) {
    case Step::FORWARD: {
      // Rumbo de referencia continuo, múltiplo de 90°. La distancia sale de la pose
      // estimada al centro de la celda destino, así el error de un tramo se corrige en el siguiente.
      float q = (float)M_PI / 2.0f;
      float a = roundf(s.th / q) * q;
      float k = roundf((pose_->theta - a) / TWO_PI_F);
      motion_->forwardTo(s.tx, s.ty, a + k * TWO_PI_F);
      break;
    }
    case Step::TURN: {
      float th = s.th;
      if (!testMode_) {
        // th es absoluto (0, 90, 180, −90): se lleva a la vuelta más cercana al rumbo actual
        float k = roundf((pose_->theta - th) / TWO_PI_F);
        th += k * TWO_PI_F;
      }
      motion_->turnTo(th);
      break;
    }
    case Step::PWM: motion_->openLoop(s.dutyL, s.dutyR, s.ms); break;
  }
  stepStarted_ = true;
}

void Mission::finishStep() {
  Step s;
  size_t n;
  xSemaphoreTake(lock_, portMAX_DELAY);
  s = steps_[idx_];
  n = steps_.size();
  xSemaphoreGive(lock_);
  if (s.goalReached != NO_GOAL) goalsReached_++;
  if (s.kind != Step::PWM) {
    lastCell_ = currentCell();
    lastHeading_ = E;
    // Rumbo cardinal más cercano al actual
    float deg = fmodf(pose_->theta * RAD_TO_DEG, 360.0f);
    if (deg < 0) deg += 360.0f;
    int q = ((int)roundf(deg / 90.0f)) & 3;  // 0 = E, 1 = N, 2 = O, 3 = S
    static const uint8_t map[4] = {E, N, O, S};
    lastHeading_ = map[q];
  }
  idx_++;
  stepStarted_ = false;
  if (idx_ >= n) {
    motion_->release();  // misión terminada: freno y EN en bajo
    state_ = State::COMPLETADO;
    message_ = testMode_ ? "prueba terminada" : "mision completada";
    testMode_ = false;
    return;
  }
  inPause_ = true;  // pausa con freno entre primitivas
  pauseT0_ = millis();
}

void Mission::step() {
  if (state_ != State::EJECUTANDO) return;
  if (motion_->faulted()) {
    fail(motion_->faultMessage());
    return;
  }
  if (inPause_) {
    if (millis() - pauseT0_ < PRIMITIVE_PAUSE_MS) return;
    inPause_ = false;
    if (pauseRequested_) {
      pauseRequested_ = false;
      state_ = State::PAUSADO;
      message_ = "pausado";
      return;
    }
  }
  if (!stepStarted_) {
    startStep();
    return;
  }
  // Motion recibe el comando en su siguiente ciclo; DONE solo cuenta si ya lo procesó
  if (motion_->finished()) {
    motion_->stop();  // DONE → IDLE para que el siguiente done() sea de la nueva primitiva
    finishStep();
  }
}
