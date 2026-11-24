#pragma once
// MPU6050 (GY-521) con un controlador propio de pocos registros sobre Wire.
// Solo se usa el giroscopio Z (guiñada). Configuración:
//   - reloj del PLL del giroscopio X (más estable que el oscilador interno)
//   - filtro digital (DLPF) de 44 Hz, muestreo a 1 kHz
//   - giroscopio ±250 °/s (131 LSB por °/s), acelerómetro ±2 g
// Sesgo b_z: se calibra en reposo con 500 muestras al arrancar (y desde la web) y se guarda en NVS.
// Si al arrancar el robot se está moviendo, se usa el último sesgo guardado.
// Con las ruedas quietas, el sesgo se sigue ajustando despacio (trackBias).
#include <Arduino.h>
#include <Wire.h>
#include <Preferences.h>

class Mpu6050 {
 public:
  struct Config {
    int sda, scl;
    uint32_t i2cFreq;
    uint16_t i2cTimeoutMs;
    uint8_t address;      // 0x68 (AD0 en bajo) o 0x69
    float yawSign;        // +1 si el eje Z del sensor apunta hacia arriba
    uint16_t calSamples;  // 500
    float calMaxStdDps;   // desviación máxima aceptada en reposo [°/s]
  };

  bool begin(const Config &c) {
    cfg_ = c;
    Wire.begin(cfg_.sda, cfg_.scl, cfg_.i2cFreq);
    Wire.setTimeOut(cfg_.i2cTimeoutMs);
    restored_ = loadBias();
    calOk_ = restored_;
    if (!init()) return false;
    // Calibración de arranque (bloquea ~1 s): 500 muestras a 2 ms
    float mean, sd;
    if (sampleBlocking(cfg_.calSamples, 2, mean, sd) && sd <= cfg_.calMaxStdDps) {
      biasDps_ = mean;
      calOk_ = true;
      saveBias();
      bootCal_ = BootCal::OK;
    } else {
      bootCal_ = restored_ ? BootCal::RESTORED : BootCal::NONE;
    }
    return true;
  }

  // Lectura a 100 Hz desde la tarea de control. Devuelve false si no hay dato válido.
  bool update() {
    if (!ok_) {
      // Sensor perdido (cable flojo): reintento cada 2 s
      if (millis() - lastRetryMs_ > 2000) {
        lastRetryMs_ = millis();
        resets_++;
        init();
      }
      return false;
    }
    int16_t raw;
    if (!readGyroZ(raw)) {
      if (++failures_ >= 10) {  // 100 ms seguidos sin respuesta
        failures_ = 0;
        ok_ = false;
        lastRetryMs_ = millis();
      }
      return false;
    }
    failures_ = 0;
    rawDps_ = raw / LSB_PER_DPS;

    // Calibración pedida desde la web: acumula calSamples lecturas (5 s a 100 Hz)
    if (calibrating_) {
      calSum_ += rawDps_;
      calSum2_ += (double)rawDps_ * rawDps_;
      if (++calN_ >= cfg_.calSamples) {
        float mean = calSum_ / calN_;
        float var = calSum2_ / calN_ - (double)mean * mean;
        float sd = var > 0 ? sqrtf(var) : 0;
        calibrating_ = false;
        lastCalStd_ = sd;
        if (sd <= cfg_.calMaxStdDps) {
          biasDps_ = mean;
          calOk_ = true;
          calSaved_ = saveBias();
          calResult_ = 1;
        } else {
          calResult_ = -1;  // se movió: no se cambia el sesgo
        }
      }
    }

    yawRateDps_ = cfg_.yawSign * (rawDps_ - biasDps_);
    lastGoodMs_ = millis();
    return true;
  }

  // Ajuste lento del sesgo con el robot quieto (lo llama Motion con las ruedas detenidas).
  void trackBias() {
    if (!ok_ || calibrating_) return;
    float e = rawDps_ - biasDps_;
    if (fabsf(e) < 1.0f) biasDps_ += 0.002f * e;  // constante de tiempo ≈ 5 s a 100 Hz
  }

  float yawRate() const { return yawRateDps_ * DEG_TO_RAD; }  // rad/s, antihorario positivo
  float biasDps() const { return biasDps_; }

  bool ok() const { return ok_; }
  bool fresh(uint32_t maxAgeMs = 100) const { return ok_ && (millis() - lastGoodMs_) < maxAgeMs; }
  uint32_t resets() const { return resets_; }
  uint8_t whoAmI() const { return who_; }

  // Calibración desde la web (en la tarea de control, porque todo el I²C ocurre ahí)
  void startCalibration() {
    calN_ = 0;
    calSum_ = calSum2_ = 0;
    calSaved_ = false;
    calResult_ = 0;
    calibrating_ = ok_;
  }
  bool calibrating() const { return calibrating_; }
  bool calibrationSaved() const { return calSaved_; }
  int8_t calibrationResult() const { return calResult_; }  // 1 bien, −1 se movió, 0 sin resultado
  float lastCalibrationStd() const { return lastCalStd_; }
  bool calibrated() const { return calOk_; }

  enum class BootCal : uint8_t { NONE, OK, RESTORED };
  BootCal bootCalibration() const { return bootCal_; }
  bool biasRestored() const { return restored_; }

 private:
  static constexpr float LSB_PER_DPS = 131.0f;  // ±250 °/s
  static constexpr uint8_t REG_SMPLRT_DIV = 0x19, REG_CONFIG = 0x1A, REG_GYRO_CONFIG = 0x1B,
                           REG_ACCEL_CONFIG = 0x1C, REG_INT_ENABLE = 0x38, REG_GYRO_ZOUT_H = 0x47,
                           REG_PWR_MGMT_1 = 0x6B, REG_WHO_AM_I = 0x75;

  bool init() {
    ok_ = false;
    if (!writeReg(REG_PWR_MGMT_1, 0x80)) return false;  // reinicio del dispositivo
    delay(100);
    uint8_t who = 0;
    if (!readReg(REG_WHO_AM_I, who)) return false;
    who_ = who;  // 0x68 en el original; algunos clones dan 0x70, 0x72 o 0x98 y funcionan igual
    bool w = writeReg(REG_PWR_MGMT_1, 0x01) &&   // despierto, reloj PLL del giroscopio X
             writeReg(REG_CONFIG, 0x03) &&       // DLPF 44 Hz (giro) / 1 kHz de muestreo
             writeReg(REG_SMPLRT_DIV, 0x00) &&   // 1 kHz / (1 + 0)
             writeReg(REG_GYRO_CONFIG, 0x00) &&  // ±250 °/s
             writeReg(REG_ACCEL_CONFIG, 0x00) && // ±2 g
             writeReg(REG_INT_ENABLE, 0x01);     // INT (GPIO 7) en dato listo; no se usa por ahora
    if (!w) return false;
    delay(50);  // el PLL tarda en estabilizarse
    failures_ = 0;
    lastGoodMs_ = millis();
    ok_ = true;
    return true;
  }

  bool sampleBlocking(uint16_t n, uint16_t periodMs, float &mean, float &sd) {
    double s = 0, s2 = 0;
    uint16_t got = 0;
    for (uint16_t i = 0; i < n; i++) {
      int16_t raw;
      if (readGyroZ(raw)) {
        float v = raw / LSB_PER_DPS;
        s += v;
        s2 += (double)v * v;
        got++;
      }
      delay(periodMs);
    }
    if (got < n / 2) return false;
    mean = s / got;
    double var = s2 / got - (double)mean * mean;
    sd = var > 0 ? sqrt(var) : 0;
    lastCalStd_ = sd;
    return true;
  }

  bool readGyroZ(int16_t &v) {
    Wire.beginTransmission(cfg_.address);
    Wire.write(REG_GYRO_ZOUT_H);
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom((int)cfg_.address, 2) != 2) return false;
    uint8_t hi = Wire.read(), lo = Wire.read();
    v = (int16_t)((hi << 8) | lo);
    return true;
  }

  bool writeReg(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(cfg_.address);
    Wire.write(reg);
    Wire.write(val);
    return Wire.endTransmission() == 0;
  }

  bool readReg(uint8_t reg, uint8_t &val) {
    Wire.beginTransmission(cfg_.address);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom((int)cfg_.address, 1) != 1) return false;
    val = Wire.read();
    return true;
  }

  bool loadBias() {
    Preferences p;
    if (!p.begin("imu", true)) return false;
    bool has = p.isKey("bias_z");
    if (has) biasDps_ = p.getFloat("bias_z", 0);
    p.end();
    return has;
  }

  bool saveBias() {
    Preferences p;
    if (!p.begin("imu", false)) return false;
    p.putFloat("bias_z", biasDps_);
    p.end();
    return true;
  }

  Config cfg_{};
  bool ok_ = false;
  bool restored_ = false;
  bool calOk_ = false;
  BootCal bootCal_ = BootCal::NONE;
  uint8_t who_ = 0;
  float biasDps_ = 0, rawDps_ = 0, yawRateDps_ = 0, lastCalStd_ = 0;
  uint32_t lastRetryMs_ = 0, lastGoodMs_ = 0, resets_ = 0;
  uint8_t failures_ = 0;

  volatile bool calibrating_ = false;
  volatile bool calSaved_ = false;
  volatile int8_t calResult_ = 0;
  uint16_t calN_ = 0;
  double calSum_ = 0, calSum2_ = 0;
};
