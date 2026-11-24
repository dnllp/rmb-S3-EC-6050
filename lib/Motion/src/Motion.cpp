#include "Motion.h"

namespace {
inline float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
inline float signf(float v) { return v >= 0 ? 1.0f : -1.0f; }
constexpr float DEG = (float)M_PI / 180.0f;
}  // namespace

const char *Motion::stateName(State s) {
  switch (s) {
    case State::IDLE: return "inactivo";
    case State::WAIT_STILL: return "esperando_quieto";
    case State::FORWARD: return "avanzando";
    case State::TURN: return "girando";
    case State::BRAKING: return "frenando";
    case State::DONE: return "terminado";
    case State::FAULT: return "falla";
    case State::OPEN_LOOP: return "prueba_pwm";
  }
  return "?";
}

void Motion::begin(const Hw &hw) {
  hw_ = hw;
  enableMotors(false);  // rueda libre hasta la primera primitiva
  brakeAll();
}

// Activado: el PWM y el freno activo funcionan. Desactivado: EN en bajo, rueda libre.
void Motion::enableMotors(bool on) {
  hw_.ml->enable(on);
  hw_.mr->enable(on);
  enabled_ = on;
}

void Motion::release() {
  portENTER_CRITICAL(&mux_);
  pending_ = makeCommand(Kind::RELEASE, 0, 0, 0, 0);
  hasPending_ = true;
  issued_++;
  portEXIT_CRITICAL(&mux_);
}

// ---------------------------------------------------------------- comandos
// (C++11 no permite inicializar con llaves un struct con valores por defecto)
Motion::Command Motion::makeCommand(Kind k, float a, float b, float c, uint32_t ms) {
  Command cmd;
  cmd.kind = k;
  cmd.a = a;
  cmd.b = b;
  cmd.c = c;
  cmd.ms = ms;
  return cmd;
}

void Motion::forwardTo(float tx, float ty, float headingRad) {
  portENTER_CRITICAL(&mux_);
  pending_ = makeCommand(Kind::FORWARD, tx, ty, headingRad, 0);
  hasPending_ = true;
  issued_++;
  portEXIT_CRITICAL(&mux_);
}

void Motion::turnTo(float headingRad) {
  portENTER_CRITICAL(&mux_);
  pending_ = makeCommand(Kind::TURN, headingRad, 0, 0, 0);
  hasPending_ = true;
  issued_++;
  portEXIT_CRITICAL(&mux_);
}

void Motion::openLoop(float dutyL, float dutyR, uint32_t ms) {
  portENTER_CRITICAL(&mux_);
  pending_ = makeCommand(Kind::OPEN_LOOP, dutyL, dutyR, 0, ms);
  hasPending_ = true;
  issued_++;
  portEXIT_CRITICAL(&mux_);
}

void Motion::stop() {
  portENTER_CRITICAL(&mux_);
  pending_ = makeCommand(Kind::STOP, 0, 0, 0, 0);
  hasPending_ = true;
  issued_++;
  portEXIT_CRITICAL(&mux_);
}

void Motion::emergency() {
  // Se actúa ya, sin esperar al siguiente ciclo de control
  hw_.ml->brake();
  hw_.mr->brake();
  portENTER_CRITICAL(&mux_);
  pending_ = makeCommand(Kind::EMERGENCY, 0, 0, 0, 0);
  hasPending_ = true;
  issued_++;
  portEXIT_CRITICAL(&mux_);
}

void Motion::clearFault() {
  if (state_ == State::FAULT) {
    fault_ = "";
    state_ = State::IDLE;
  }
}

void Motion::setFault(const char *msg) {
  brakeAll();
  fault_ = msg;
  kind_ = Kind::NONE;
  state_ = State::FAULT;
}

Motion::Telemetry Motion::telemetry() const {
  Telemetry t;
  t.state = state_;
  t.vL = vL_;
  t.vR = vR_;
  t.dutyL = dutyL_;
  t.dutyR = dutyR_;
  t.ticksL = hw_.el->total();
  t.ticksR = hw_.er->total();
  t.progress = progress_;
  t.slipEvents = hw_.pose->slipEvents;
  t.brakeDist = brakeDist_;
  t.imuOk = imuOk_;
  return t;
}

void Motion::applyCommand(const Command &c) {
  switch (c.kind) {
    case Kind::EMERGENCY:
    case Kind::RELEASE:
      brakeAll();
      if (enabled_) delay(20);  // un instante de freno activo antes de soltar
      enableMotors(false);
      dutyL_ = dutyR_ = 0;
      kind_ = Kind::NONE;
      if (c.kind == Kind::EMERGENCY || state_ != State::FAULT) state_ = State::IDLE;
      return;
    case Kind::STOP:
      brakeAll();
      kind_ = Kind::NONE;
      state_ = State::IDLE;
      return;
    case Kind::FORWARD:
    case Kind::TURN:
    case Kind::OPEN_LOOP:
      if (state_ == State::FAULT) return;  // hay que limpiar la falla primero
      kind_ = c.kind;
      if (c.kind == Kind::FORWARD) {
        tx_ = c.a;
        ty_ = c.b;
        thRef_ = c.c;
      } else if (c.kind == Kind::TURN) {
        thRef_ = c.a;
      } else {
        dutyLCmd_ = clampf(c.a, -1, 1);
        dutyRCmd_ = clampf(c.b, -1, 1);
        openLoopMs_ = c.ms;
      }
      enableMotors(true);  // el freno activo y el PWM ya funcionan
      brakeAll();
      state_ = State::WAIT_STILL;  // cada primitiva arranca desde el reposo
      settleT0_ = millis();
      return;
    default:
      return;
  }
}

// ---------------------------------------------------------------- ciclo de control
void Motion::step(float dt) {
  const Settings &s = gSettings;
  const float mpt = s.metersPerTick();

  // 1) Sensores: encoders, giroscopio Z y pose fusionada
  int32_t dnL = hw_.el->update(dt);
  int32_t dnR = hw_.er->update(dt);
  absTicks_ += (dnL < 0 ? -dnL : dnL) + (dnR < 0 ? -dnR : dnR);
  imuOk_ = hw_.imu->update() && hw_.imu->fresh(100);
  bool still = hw_.el->isStill(ENC_STILL_MS) && hw_.er->isStill(ENC_STILL_MS) && dutyL_ == 0 && dutyR_ == 0;
  if (still && imuOk_) hw_.imu->trackBias();
  hw_.pose->update(dnL, dnR, imuOk_ ? hw_.imu->yawRate() : 0.0f, imuOk_, still, dt, mpt, s.wheelBase,
                   s.gyroBeta);
  vL_ = hw_.el->ticksPerSecond() * mpt;
  vR_ = hw_.er->ticksPerSecond() * mpt;

  // 2) Comandos pendientes
  if (hasPending_) {
    Command c;
    portENTER_CRITICAL(&mux_);
    c = pending_;
    hasPending_ = false;
    processed_ = issued_;
    portEXIT_CRITICAL(&mux_);
    applyCommand(c);
  }

  // 3) Primitiva
  bool moving = state_ == State::FORWARD || state_ == State::TURN || state_ == State::WAIT_STILL;
  if (moving && kind_ != Kind::OPEN_LOOP) {
    // Con β = 0 (solo encoders) se puede trabajar sin la MPU6050
    if (s.gyroBeta > 0 && !imuOk_) {
      setFault("IMU sin datos (revisa el cable de la MPU6050)");
      return;
    }
    if (s.gyroBeta > 0 && !hw_.imu->calibrated()) {
      setFault("calibra el giroscopio (robot quieto) antes de mover");
      return;
    }
  }
  if ((state_ == State::FORWARD || state_ == State::TURN) && millis() - t0_ > timeoutMs_) {
    setFault(state_ == State::FORWARD ? "tiempo agotado al avanzar" : "tiempo agotado al girar");
    return;
  }

  switch (state_) {
    case State::WAIT_STILL:
      brakeAll();
      if (hw_.el->isStill(ENC_STILL_MS) && hw_.er->isStill(ENC_STILL_MS) &&
          millis() - settleT0_ >= ENC_STILL_MS) {
        beginPrimitive();
      } else if (millis() - settleT0_ > 3000) {
        setFault("las ruedas no se detienen");
      }
      break;
    case State::FORWARD: stepForward(dt); break;
    case State::TURN: stepTurn(dt); break;
    case State::BRAKING: stepBraking(); break;
    case State::OPEN_LOOP:
      dutyL_ = dutyLCmd_;
      dutyR_ = dutyRCmd_;
      hw_.ml->setDuty(dutyL_);
      hw_.mr->setDuty(dutyR_);
      if ((int32_t)(millis() - openLoopEnd_) >= 0) {
        brakeAll();
        state_ = State::DONE;
      }
      break;
    default:
      break;
  }
}

void Motion::beginPrimitive() {
  const Settings &s = gSettings;
  Pose &p = *hw_.pose;
  t0_ = millis();
  piL_.reset();
  piR_.reset();
  vCmd_ = 0;
  wCmd_ = 0;
  accelScale_ = 1;
  progress_ = 0;

  if (kind_ == Kind::FORWARD) {
    float ux = cosf(thRef_), uy = sinf(thRef_);
    total_ = (tx_ - p.x) * ux + (ty_ - p.y) * uy;
    p.resetSlip();
    p.slipWatch = true;
    float vmin = s.vApproach > 0.02f ? s.vApproach : 0.02f;
    timeoutMs_ = (uint32_t)(1000.0f * (fabsf(total_) / vmin + 3.0f));
    state_ = State::FORWARD;
    if (total_ <= 0.002f) {  // ya estamos (o nos pasamos): solo frenar
      brakeAll();
      brakeTicks0_ = forwardTicks();
      state_ = State::BRAKING;
    }
  } else if (kind_ == Kind::TURN) {
    float e = thRef_ - p.theta;
    total_ = e;
    p.slipWatch = false;
    turnTicks0_ = absTicks_;
    turnExpectedTicks_ = 2.0f * (s.wheelBase * 0.5f * fabsf(e)) / s.metersPerTick();
    turnSettling_ = false;
    timeoutMs_ = 6000;
    state_ = State::TURN;
  } else if (kind_ == Kind::OPEN_LOOP) {
    p.slipWatch = false;
    openLoopEnd_ = millis() + openLoopMs_;
    state_ = State::OPEN_LOOP;
  }
}

void Motion::stepForward(float dt) {
  const Settings &s = gSettings;
  Pose &p = *hw_.pose;
  float ux = cosf(thRef_), uy = sinf(thRef_);
  float rem = (tx_ - p.x) * ux + (ty_ - p.y) * uy;  // distancia restante sobre la línea
  float left = -(p.x - tx_) * uy + (p.y - ty_) * ux;  // desplazamiento a la izquierda de la línea
  progress_ = total_ > 0 ? clampf(1.0f - rem / total_, 0, 1) : 1;

  // Corte: distancia restante < 5 mm + lo que se desliza al frenar
  if (rem < FORWARD_STOP_M + brakeDist_) {
    brakeAll();
    brakeTicks0_ = forwardTicks();
    state_ = State::BRAKING;
    return;
  }

  // Deslizamiento: rumbo de encoders vs giroscopio
  if (fabsf(p.slipError()) > SLIP_LIMIT_DEG * DEG) {
    p.slipEvents++;
    p.resetSlip();
    accelScale_ = 0.5f;
  }

  // Perfil trapezoidal con zona de aproximación
  float a = s.accel * accelScale_;
  float vTarget = s.vMax;
  float vDecel = sqrtf(s.vApproach * s.vApproach + 2.0f * a * fmaxf(0.0f, rem - APPROACH_DIST));
  if (vDecel < vTarget) vTarget = vDecel;
  if (rem < APPROACH_DIST) vTarget = s.vApproach;
  if (vTarget > vCmd_) vCmd_ = fminf(vTarget, vCmd_ + a * dt);
  else vCmd_ = vTarget;  // frenar en el perfil es inmediato

  // Rumbo y error lateral
  float eTh = thRef_ - p.theta;
  float eY = -left;
  float lateralGain = clampf(vCmd_ / 0.1f, 0, 1);
  float w = s.headingK * eTh + s.lateralK * eY * lateralGain;
  w = clampf(w, -1.0f, 1.0f);
  float vl = vCmd_ - 0.5f * s.wheelBase * w;
  float vr = vCmd_ + 0.5f * s.wheelBase * w;
  driveWheels(vl, vr, dt);
}

void Motion::stepTurn(float dt) {
  const Settings &s = gSettings;
  Pose &p = *hw_.pose;
  float e = thRef_ - p.theta;
  float wMeas = imuOk_ ? hw_.imu->yawRate() : p.lastRate;
  progress_ = fabsf(total_) > 1e-3f ? clampf(1.0f - fabsf(e) / fabsf(total_), 0, 1) : 1;

  // Término: |eθ| < tolerancia y |ω| < TURN_STILL durante TURN_SETTLE_MS.
  // Dentro de la tolerancia se frena; si se sale (se pasó), se corrige en cualquier sentido.
  if (fabsf(e) < TURN_TOL_DEG * DEG) {
    brakeAll();
    wCmd_ = 0;
    if (fabsf(wMeas) < TURN_STILL_DEG_S * DEG) {
      if (!turnSettling_) {
        turnSettling_ = true;
        settleT0_ = millis();
      } else if (millis() - settleT0_ >= TURN_SETTLE_MS) {
        progress_ = 1;
        kind_ = Kind::NONE;
        state_ = State::DONE;
      }
    } else {
      turnSettling_ = false;
    }
    return;
  }
  turnSettling_ = false;

  // El encoder da la referencia secundaria: si las ruedas giraron mucho más de lo esperado, algo falla
  int64_t ticks = absTicks_ - turnTicks0_;
  if (turnExpectedTicks_ > 50 && ticks > 3.0f * turnExpectedTicks_ + 200) {
    setFault("el giro no converge (¿IMU_YAW_SIGN al revés?)");
    return;
  }

  // Perfil trapezoidal en ω + PD sobre el rumbo
  float wMax = s.turnWMaxDeg * DEG, alpha = s.turnAlphaDeg * DEG;
  float wLim = fminf(wMax, sqrtf(2.0f * alpha * fabsf(e)));
  wLim = fminf(wLim, fabsf(wCmd_) + alpha * dt);
  float w = s.turnKp * e - s.turnKd * wMeas;
  float wMin = TURN_W_MIN_DEG * DEG;
  float mag = clampf(fabsf(w), wMin, fmaxf(wLim, wMin));
  wCmd_ = signf(e) * mag;
  float vr = 0.5f * s.wheelBase * wCmd_;
  driveWheels(-vr, vr, dt);
}

void Motion::stepBraking() {
  brakeAll();
  bool still = hw_.el->isStill(ENC_STILL_MS) && hw_.er->isStill(ENC_STILL_MS);
  if (!still) return;
  if (kind_ == Kind::FORWARD) {
    // Lo que se deslizó tras el freno: se aprende para cortar antes la próxima vez
    int64_t extra = forwardTicks() - brakeTicks0_;
    float d = 0.5f * (float)extra * gSettings.metersPerTick();
    brakeDist_ = 0.7f * brakeDist_ + 0.3f * clampf(d, 0.0f, 0.05f);
    hw_.pose->slipWatch = false;
  }
  progress_ = 1;
  kind_ = Kind::NONE;
  state_ = State::DONE;
}

// Suma de cuentas con signo de las dos ruedas (avance)
int64_t Motion::forwardTicks() const { return hw_.el->total() + hw_.er->total(); }

float Motion::WheelPI::out(float vref, float vmeas, float dt, const Settings &s) {
  if (fabsf(vref) < 1e-4f) {
    integ = 0;
    return 0;
  }
  // Sin monitor de batería: la prealimentación no se corrige por voltaje; el PI lo absorbe
  float ff = signf(vref) * (s.pwmMin + s.wheelKff * fabsf(vref));
  float e = vref - vmeas;
  float u = ff + s.wheelKp * e + integ;
  float uSat = clampf(u, -1, 1);
  // Anti-windup: solo integrar si no está saturado o si el error ayuda a salir
  if (u == uSat || (u > 1 && e < 0) || (u < -1 && e > 0)) integ += s.wheelKi * e * dt;
  integ = clampf(integ, -0.5f, 0.5f);
  return uSat;
}

void Motion::driveWheels(float vrefL, float vrefR, float dt) {
  const Settings &s = gSettings;
  // Con cuadratura el signo de la velocidad medida es real: cada rueda puede invertir el sentido.
  float dl = piL_.out(vrefL, vL_, dt, s);
  float dr = piR_.out(vrefR, vR_, dt, s);
  dutyL_ = dl;
  dutyR_ = dr;
  if (dl == 0) hw_.ml->brake(); else hw_.ml->setDuty(dl);
  if (dr == 0) hw_.mr->brake(); else hw_.mr->setDuty(dr);
}

void Motion::brakeAll() {
  hw_.ml->brake();
  hw_.mr->brake();
  dutyL_ = dutyR_ = 0;
  piL_.reset();
  piR_.reset();
}
