#pragma once
// Seguridad cada 20 ms. Sin monitor de batería ni zumbador:
// - Atasco: PWM alto y sin pulsos del encoder durante STALL_TIME_MS.
// - Latido del navegador.
#include <Arduino.h>
#include "Bts7960.h"
#include "QuadEncoder.h"
#include "roomba_settings.h"

class Safety {
 public:
  enum class Event : uint8_t { NONE, STALL, HEARTBEAT_LOST };

  void begin(Bts7960 *ml, Bts7960 *mr, QuadEncoder *el, QuadEncoder *er) {
    ml_ = ml;
    mr_ = mr;
    el_ = el;
    er_ = er;
    lastBeatMs_ = millis();
  }

  // Devuelve el evento detectado en este ciclo (o NONE).
  // 'running' = hay una misión o primitiva en movimiento.
  Event step(bool running) {
    uint32_t now = millis();
    Event ev = Event::NONE;

    // Atasco: la rueda recibe PWM pero el encoder no da pulsos
    bool stalledL = fabsf(ml_->duty()) > STALL_MIN_DUTY && el_->isStill(STALL_TIME_MS);
    bool stalledR = fabsf(mr_->duty()) > STALL_MIN_DUTY && er_->isStill(STALL_TIME_MS);
    if (running && (stalledL || stalledR)) {
      if (!stallT0_) stallT0_ = now;
      if (now - stallT0_ >= STALL_TIME_MS) ev = Event::STALL;
    } else {
      stallT0_ = 0;
    }

    // Latido del navegador
    if (ev == Event::NONE && running && gSettings.heartbeatStop && beatSeen_ &&
        now - lastBeatMs_ > HEARTBEAT_TIMEOUT_MS)
      ev = Event::HEARTBEAT_LOST;

    if (ev != Event::NONE) lastEvent_ = ev;
    return ev;
  }

  void heartbeat() {
    lastBeatMs_ = millis();
    beatSeen_ = true;
  }
  // Sin clientes conectados no se exige latido hasta que llegue el primero
  void forgetHeartbeat() { beatSeen_ = false; }

  Event lastEvent() const { return lastEvent_; }

  static const char *eventMessage(Event e) {
    switch (e) {
      case Event::STALL: return "posible choque o rueda bloqueada (sin cuentas del encoder)";
      case Event::HEARTBEAT_LOST: return "se perdio la conexion con el navegador";
      default: return "";
    }
  }

 private:
  Bts7960 *ml_ = nullptr, *mr_ = nullptr;
  QuadEncoder *el_ = nullptr, *er_ = nullptr;
  uint32_t stallT0_ = 0;
  uint32_t lastBeatMs_ = 0;
  bool beatSeen_ = false;
  Event lastEvent_ = Event::NONE;
};
