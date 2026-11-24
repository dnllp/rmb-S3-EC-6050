#pragma once
// Encoder de cuadratura (canales A y B) con el contador por hardware PCNT, mediante la
// biblioteca ESP32Encoder. Decodificación ×4: cuenta los dos flancos de los dos canales,
// y el signo sale del propio encoder (no hace falta suponerlo por el comando del motor).
//
// La velocidad se calcula con la diferencia de cuentas en una ventana deslizante de
// VEL_WINDOW ciclos de control (50 ms a 100 Hz), que suaviza sin retrasar demasiado.
#include <Arduino.h>
#include <ESP32Encoder.h>
#include <esp_timer.h>

class QuadEncoder {
 public:
  // pinA, pinB: entradas. invert: true si al avanzar la rueda la cuenta baja.
  // filter: filtro de glitches del PCNT en ciclos de APB (80 MHz), máximo 1023 (~12.8 µs).
  // pullUp: activa el pull-up interno del ESP32-S3 (todos sus GPIO lo tienen).
  bool begin(int pinA, int pinB, bool invert, uint16_t filter, bool pullUp) {
    invert_ = invert;
    ESP32Encoder::useInternalWeakPullResistors = pullUp ? puType::up : puType::none;
    enc_.attachFullQuad(pinA, pinB);
    if (!enc_.isAttached()) return false;
    enc_.setFilter(filter);
    enc_.clearCount();
    last_ = 0;
    lastChangeUs_ = 0;
    for (auto &h : hist_) h = 0;
    return true;
  }

  // Llamar en cada ciclo de control. Devuelve el incremento con signo, en cuentas.
  int32_t update(float dt) {
    int64_t c = enc_.getCount();
    if (invert_) c = -c;
    int32_t d = (int32_t)(c - last_);
    last_ = c;
    if (d != 0) lastChangeUs_ = esp_timer_get_time();

    // Ventana deslizante: cuentas de los últimos VEL_WINDOW ciclos
    histSum_ += d - hist_[histIdx_];
    hist_[histIdx_] = d;
    histIdx_ = (histIdx_ + 1) % VEL_WINDOW;
    if (histFill_ < VEL_WINDOW) histFill_++;
    ticksPerSec_ = dt > 0 ? (float)histSum_ / (histFill_ * dt) : 0;
    return d;
  }

  float ticksPerSecond() const { return ticksPerSec_; }
  int64_t total() const { return last_; }  // cuentas con signo desde el arranque

  uint32_t usSinceLastPulse() const {
    if (lastChangeUs_ == 0) return UINT32_MAX;
    int64_t d = esp_timer_get_time() - lastChangeUs_;
    return d > (int64_t)UINT32_MAX ? UINT32_MAX : (uint32_t)d;
  }
  bool isStill(uint32_t ms) const { return usSinceLastPulse() >= ms * 1000u; }

 private:
  static constexpr uint8_t VEL_WINDOW = 5;

  ESP32Encoder enc_;
  bool invert_ = false;
  int64_t last_ = 0;
  volatile int64_t lastChangeUs_ = 0;
  int32_t hist_[VEL_WINDOW] = {};
  int32_t histSum_ = 0;
  uint8_t histIdx_ = 0, histFill_ = 0;
  float ticksPerSec_ = 0;
};
