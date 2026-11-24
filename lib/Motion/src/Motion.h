#pragma once
// Control de movimiento a 100 Hz: lee encoders y giroscopio, actualiza la pose y ejecuta
// las dos primitivas (avanzar hasta un punto y girar en sitio) con frenado activo del BTS7960.
// Con encoders de cuadratura cada rueda puede invertir el sentido dentro de una primitiva.
#include <Arduino.h>
#include "Bts7960.h"
#include "QuadEncoder.h"
#include "Mpu6050.h"
#include "Pose.h"
#include "roomba_settings.h"

class Motion {
 public:
  enum class State : uint8_t { IDLE, WAIT_STILL, FORWARD, TURN, BRAKING, DONE, FAULT, OPEN_LOOP };

  struct Hw {
    Bts7960 *ml, *mr;
    QuadEncoder *el, *er;
    Mpu6050 *imu;
    Pose *pose;
  };

  struct Telemetry {
    State state;
    float vL, vR;        // m/s medidas
    float dutyL, dutyR;  // −1..1
    int64_t ticksL, ticksR;
    float progress;      // 0..1 de la primitiva en curso
    uint32_t slipEvents;
    float brakeDist;     // m, distancia de frenado aprendida
    bool imuOk;
  };

  void begin(const Hw &hw);

  // Tarea de control: llamar cada CONTROL_PERIOD_MS
  void step(float dt);

  // Comandos (cualquier tarea)
  void forwardTo(float tx, float ty, float headingRad);  // avanzar hasta (tx, ty) con rumbo fijo
  void turnTo(float headingRad);                         // girar en sitio hasta el rumbo absoluto
  void openLoop(float dutyL, float dutyR, uint32_t ms);  // prueba de motores (curva PWM)
  void stop();                                           // frenar y cancelar
  void emergency();                                      // frenar y dejar en rueda libre de inmediato
  void release();                                        // frenar un instante y soltar (fin de misión)
  bool motorsEnabled() const { return enabled_; }
  void clearFault();

  State state() const { return state_; }
  bool done() const { return state_ == State::DONE; }
  // DONE de la última orden enviada (no de una anterior que aún no se procesa)
  bool finished() const { return processed_ == issued_ && !hasPending_ && state_ == State::DONE; }
  bool faulted() const { return state_ == State::FAULT; }
  bool idleOrDone() const { return state_ == State::IDLE || state_ == State::DONE; }
  const char *faultMessage() const { return fault_; }
  void setFault(const char *msg);
  Telemetry telemetry() const;

  static const char *stateName(State s);

 private:
  enum class Kind : uint8_t { NONE, FORWARD, TURN, OPEN_LOOP, STOP, EMERGENCY, RELEASE };
  struct Command {
    Kind kind = Kind::NONE;
    float a = 0, b = 0, c = 0;
    uint32_t ms = 0;
  };

  struct WheelPI {
    float integ = 0;
    void reset() { integ = 0; }
    float out(float vref, float vmeas, float dt, const Settings &s);
  };

  static Command makeCommand(Kind k, float a, float b, float c, uint32_t ms);
  void applyCommand(const Command &c);
  void beginPrimitive();
  void stepForward(float dt);
  void stepTurn(float dt);
  void stepBraking();
  void driveWheels(float vrefL, float vrefR, float dt);
  void brakeAll();
  void enableMotors(bool on);
  int64_t forwardTicks() const;

  Hw hw_{};
  volatile State state_ = State::IDLE;
  const char *fault_ = "";
  portMUX_TYPE mux_ = portMUX_INITIALIZER_UNLOCKED;
  Command pending_;
  volatile bool hasPending_ = false;
  volatile uint32_t issued_ = 0, processed_ = 0;

  // Primitiva en curso
  Kind kind_ = Kind::NONE;
  float tx_ = 0, ty_ = 0, thRef_ = 0;
  float total_ = 0;  // distancia [m] o ángulo [rad] inicial
  float vCmd_ = 0, wCmd_ = 0;
  float accelScale_ = 1;
  uint32_t t0_ = 0, timeoutMs_ = 0, settleT0_ = 0;
  int64_t brakeTicks0_ = 0;
  float brakeDist_ = 0;   // EMA de la distancia recorrida tras frenar
  int64_t turnTicks0_ = 0;
  float turnExpectedTicks_ = 0;
  bool turnSettling_ = false;
  int64_t absTicks_ = 0;  // suma de |Δcuentas| de las dos ruedas desde el arranque
  float dutyL_ = 0, dutyR_ = 0;
  float dutyLCmd_ = 0, dutyRCmd_ = 0;  // prueba de PWM en lazo abierto
  uint32_t openLoopEnd_ = 0, openLoopMs_ = 0;
  float vL_ = 0, vR_ = 0;
  float progress_ = 0;
  bool imuOk_ = false;
  volatile bool enabled_ = false;
  WheelPI piL_, piR_;
};
