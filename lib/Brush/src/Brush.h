#pragma once
// Barredora (cepillo) encendida/apagada por un GPIO hacia un módulo MOSFET o relé.
// Modo automático: gira solo mientras la misión está EJECUTANDO.
// Modo manual (prueba desde la web): solo con el robot sin misión en curso; se cancela
// en cuanto cambia el estado de la misión.
#include <Arduino.h>

class Brush {
 public:
  void begin(int pin, bool activeHigh) {
    pin_ = pin;
    activeHigh_ = activeHigh;
    pinMode(pin_, OUTPUT);
    write(false);
  }

  // Llamar desde la tarea de misión. missionRunning = estado EJECUTANDO.
  void update(bool autoMode, bool missionRunning, bool stateChanged) {
    if (stateChanged || missionRunning) manual_ = false;
    write((autoMode && missionRunning) || manual_);
  }

  void setManual(bool on) { manual_ = on; }
  bool on() const { return on_; }
  bool manual() const { return manual_; }

 private:
  void write(bool on) {
    on_ = on;
    digitalWrite(pin_, (on == activeHigh_) ? HIGH : LOW);
  }

  int pin_ = -1;
  bool activeHigh_ = true;
  volatile bool manual_ = false;
  volatile bool on_ = false;
};
