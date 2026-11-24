#pragma once
// Un módulo BTS7960 (IBT-2) por motor, con su propio pin EN.
//
//   EN   RPWM  LPWM   resultado
//   1    PWM   0      adelante
//   1    0     PWM    atrás
//   1    0     0      freno activo (los dos lados bajos conducen)
//   0    x     x      rueda libre (las dos medias H apagadas)
//
// En el módulo IBT-2 une R_EN y L_EN y llévalos al pin EN de la ESP32-S3.
// Pon una resistencia de 10 kΩ de EN a GND: mientras la ESP32 arranca (pines flotando)
// los motores quedan en rueda libre. Los pines R_IS y L_IS no se usan.
// El duty se limita a maxDuty para no pasar del voltaje nominal del motor (ver roomba_config.h).
// La interfaz es la misma que la del L298n de la Propuesta A: Motion y Safety no cambian.
#include <Arduino.h>

class Bts7960 {
 public:
  struct Pins {
    int rpwm, lpwm, en;
    uint8_t chR, chL;  // canales LEDC
  };

  void begin(const Pins &p, uint32_t freqHz, uint8_t bits, bool invert, float maxDuty) {
    p_ = p;
    invert_ = invert;
    maxDuty_ = maxDuty > 1.0f ? 1.0f : (maxDuty < 0.1f ? 0.1f : maxDuty);
    fullScale_ = (1u << bits) - 1;
    pinMode(p_.en, OUTPUT);
    digitalWrite(p_.en, LOW);
    ledcSetup(p_.chR, freqHz, bits);
    ledcSetup(p_.chL, freqHz, bits);
    ledcAttachPin(p_.rpwm, p_.chR);
    ledcAttachPin(p_.lpwm, p_.chL);
    ledcWrite(p_.chR, 0);
    ledcWrite(p_.chL, 0);
    enabled_ = false;
  }

  // Sin enable() el puente queda en rueda libre: setDuty() y brake() no hacen nada.
  void enable(bool on) {
    if (on) {
      ledcWrite(p_.chR, 0);
      ledcWrite(p_.chL, 0);
      digitalWrite(p_.en, HIGH);
      enabled_ = true;
      braking_ = true;  // EN alto con RPWM = LPWM = 0 ya es freno
    } else {
      coast();
    }
  }
  bool enabled() const { return enabled_; }

  // duty en [−1, 1]. Positivo = avanzar.
  void setDuty(float duty) {
    if (!enabled_) return;
    if (duty > 1) duty = 1;
    if (duty < -1) duty = -1;
    float hw = invert_ ? -duty : duty;
    uint32_t d = (uint32_t)(fabsf(hw) * maxDuty_ * fullScale_ + 0.5f);
    if (d == 0) {
      brake();
      return;
    }
    // Primero se apaga el lado contrario y luego se enciende el que toca
    if (hw >= 0) {
      ledcWrite(p_.chL, 0);
      ledcWrite(p_.chR, d);
    } else {
      ledcWrite(p_.chR, 0);
      ledcWrite(p_.chL, d);
    }
    duty_ = duty;
    braking_ = false;
  }

  // Freno activo: RPWM = LPWM = 0 con EN alto. Con el puente desactivado queda libre.
  void brake() {
    ledcWrite(p_.chR, 0);
    ledcWrite(p_.chL, 0);
    duty_ = 0;
    braking_ = enabled_;
  }

  // Rueda libre: EN en bajo
  void coast() {
    ledcWrite(p_.chR, 0);
    ledcWrite(p_.chL, 0);
    digitalWrite(p_.en, LOW);
    enabled_ = false;
    duty_ = 0;
    braking_ = false;
  }

  float duty() const { return duty_; }  // lo que pidió el control, antes del límite maxDuty
  bool braking() const { return braking_; }

 private:
  Pins p_{};
  bool invert_ = false;
  bool enabled_ = false;
  float maxDuty_ = 1.0f;
  uint32_t fullScale_ = 1023;
  float duty_ = 0;
  bool braking_ = false;
};
