#pragma once
// Parámetros ajustables en campo (GET/POST /api/config), persistentes en NVS.
#include <Arduino.h>
#include <Preferences.h>
#include "roomba_config.h"

struct Settings {
  static constexpr uint16_t VERSION = 1;  // súbelo si cambias el struct: descarta lo guardado
  uint16_t version = VERSION;

  float ticksPerRev = CUENTAS_POR_VUELTA;
  float wheelRadius = WHEEL_RADIUS_M;
  float wheelBase = WHEEL_BASE_M;

  float wheelKp = DefaultGains::WHEEL_KP;
  float wheelKi = DefaultGains::WHEEL_KI;
  float wheelKff = DefaultGains::WHEEL_KFF;
  float pwmMin = DefaultGains::WHEEL_PWM_MIN;
  float headingK = DefaultGains::HEADING_K;
  float lateralK = DefaultGains::LATERAL_K;
  float turnKp = DefaultGains::TURN_KP;
  float turnKd = DefaultGains::TURN_KD;

  float vMax = V_MAX;
  float accel = ACCEL;
  float vApproach = V_APPROACH;
  float turnWMaxDeg = TURN_W_MAX_DEG;
  float turnAlphaDeg = TURN_ALPHA_DEG;
  float turnCost = TURN_COST;
  float gyroBeta = GYRO_BETA;  // β del filtro complementario (0 = solo encoders)

  bool heartbeatStop = true;  // frenar si el navegador deja de mandar latido
  bool brushAuto = true;      // barredora encendida mientras la misión está EJECUTANDO

  // Distancia por cuenta [m]
  float metersPerTick() const { return 2.0f * PI * wheelRadius / ticksPerRev; }

  void load() {
    Preferences p;
    if (!p.begin("settings", true)) return;
    Settings tmp;
    size_t n = p.getBytes("blob", &tmp, sizeof(tmp));
    p.end();
    if (n == sizeof(tmp) && tmp.version == VERSION) *this = tmp;
  }
  void save() const {
    Preferences p;
    if (!p.begin("settings", false)) return;
    p.putBytes("blob", this, sizeof(*this));
    p.end();
  }
  void reset() { *this = Settings(); }
};

// Instancia global (definida en main.cpp). Se lee desde varias tareas: los floats de 32 bits
// se escriben de forma atómica en la ESP32, así que basta con copiar el valor al usarlo.
extern Settings gSettings;
