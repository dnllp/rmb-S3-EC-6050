// Roomba cuadrícula — Propuesta S3
// ESP32-S3 + 2 × BTS7960 + encoders de cuadratura (PCNT) + MPU6050 + barredora + interfaz web.
// Sin monitor de batería y sin zumbador.
//
// Núcleo 1: control (100 Hz), seguridad (50 Hz), misión (20 Hz).
// Núcleo 0: Wi-Fi y servidor (AsyncTCP), planificador (bajo demanda), telemetría (10 Hz).
#include <Arduino.h>

#include "roomba_config.h"
#include "roomba_settings.h"
#include "Bts7960.h"
#include "QuadEncoder.h"
#include "Mpu6050.h"
#include "Pose.h"
#include "Motion.h"
#include "Safety.h"
#include "Mission.h"
#include "Brush.h"
#include "WebApi.h"

Settings gSettings;

static Bts7960 motorL, motorR;
static QuadEncoder encL, encR;
static Mpu6050 imu;
static Pose pose;
static Motion motion;
static Safety safety;
static Mission mission;
static Brush brush;

static volatile bool planReady = false;

// ---------------------------------------------------------------- utilidades
static bool missionRunning() {
  Mission::State s = mission.state();
  Motion::State m = motion.state();
  return s == Mission::State::EJECUTANDO ||
         m == Motion::State::FORWARD || m == Motion::State::TURN || m == Motion::State::OPEN_LOOP;
}

// ---------------------------------------------------------------- tareas
static void controlTask(void *) {
  TickType_t last = xTaskGetTickCount();
  const float dt = CONTROL_PERIOD_MS / 1000.0f;
  for (;;) {
    motion.step(dt);
    vTaskDelayUntil(&last, pdMS_TO_TICKS(CONTROL_PERIOD_MS));
  }
}

static void safetyTask(void *) {
  TickType_t last = xTaskGetTickCount();
  Safety::Event prev = Safety::Event::NONE;
  for (;;) {
    Safety::Event ev = safety.step(missionRunning());
    if (ev != Safety::Event::NONE && ev != prev) {
      const char *msg = Safety::eventMessage(ev);
      Serial.printf("[seguridad] %s\n", msg);
      if (ev == Safety::Event::HEARTBEAT_LOST) {
        // Se frena (freno activo y luego rueda libre); hay que volver a planear y ejecutar
        mission.emergencyStop();
      } else {
        motion.emergency();
        mission.fail(msg);
      }
      webapi::sendEvent(msg);
    }
    prev = ev;
    vTaskDelayUntil(&last, pdMS_TO_TICKS(SAFETY_PERIOD_MS));
  }
}

static void missionTask(void *) {
  TickType_t last = xTaskGetTickCount();
  bool btnPrev = true;
  uint32_t btnT = 0;
  Mission::State prevState = mission.state();
  for (;;) {
    mission.step();

    // Botón físico (activo en bajo): paro si está en marcha, ejecutar si hay plan, reanudar si está pausado
    bool btn = digitalRead(PIN_BUTTON);
    if (!btn && btnPrev && millis() - btnT > 300) {
      btnT = millis();
      const char *err = "";
      switch (mission.state()) {
        case Mission::State::EJECUTANDO: mission.emergencyStop(); break;
        case Mission::State::LISTO: mission.execute(true, &err); break;
        case Mission::State::PAUSADO: mission.togglePause(&err); break;
        default: motion.emergency(); break;
      }
    }
    btnPrev = btn;

    // Barredora: automática mientras la misión está EJECUTANDO (se apaga en pausa, paro o falla)
    brush.update(gSettings.brushAuto, mission.state() == Mission::State::EJECUTANDO,
                 mission.state() != prevState);

    // LED: fijo = ejecutando, parpadeo lento = listo/pausado, rápido = falla
    Mission::State s = mission.state();
    uint32_t t = millis();
    bool led = false;
    switch (s) {
      case Mission::State::EJECUTANDO: led = true; break;
      case Mission::State::LISTO:
      case Mission::State::PAUSADO: led = (t / 500) & 1; break;
      case Mission::State::FALLA: led = (t / 100) & 1; break;
      default: led = (t % 2000) < 50; break;
    }
    digitalWrite(PIN_LED, led);

    if (s != prevState) {
      Serial.printf("[mision] %s -> %s (%s)\n", Mission::stateName(prevState), Mission::stateName(s),
                    mission.message());
      prevState = s;
    }
    vTaskDelayUntil(&last, pdMS_TO_TICKS(MISSION_PERIOD_MS));
  }
}

static void plannerTask(void *) { mission.plannerLoop(); }

static void telemetryTask(void *) {
  TickType_t last = xTaskGetTickCount();
  for (;;) {
    if (planReady) {
      planReady = false;
      webapi::broadcastPlan();
    }
    webapi::sendTelemetry();
    vTaskDelayUntil(&last, pdMS_TO_TICKS(TELEMETRY_PERIOD_MS));
  }
}

// ---------------------------------------------------------------- arranque
void setup() {
  // Marca lo antes posible: si no sale en el monitor, el reinicio ocurre antes de llegar aquí
  // (alimentación o pines de arranque), no en el código del robot.
  Serial.begin(115200);
  Serial.printf("\n[roomba-grid-s3] setup() reset=%d\n", (int)esp_reset_reason());
  Serial.flush();

  // Motores en rueda libre: EN (39 y 17) y PWM en bajo.
  for (int pin : {PIN_L_EN, PIN_R_EN, PIN_L_RPWM, PIN_L_LPWM, PIN_R_RPWM, PIN_R_LPWM}) {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
  }
  // Barredora apagada según su polaridad (BRUSH_ACTIVE_HIGH). No se fuerza a LOW antes:
  // con un relé activo en bajo eso lo encendería al arrancar.
  brush.begin(PIN_BRUSH, BRUSH_ACTIVE_HIGH);

  delay(100);
  Serial.println("[roomba-grid-s3] arrancando");

  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_BUTTON, INPUT_PULLUP);

  gSettings.load();

  motorL.begin({PIN_L_RPWM, PIN_L_LPWM, PIN_L_EN, LEDC_CH_L_R, LEDC_CH_L_L}, PWM_FREQ_HZ, PWM_BITS,
               LEFT_MOTOR_INVERT, MOTOR_MAX_DUTY);
  motorR.begin({PIN_R_RPWM, PIN_R_LPWM, PIN_R_EN, LEDC_CH_R_R, LEDC_CH_R_L}, PWM_FREQ_HZ, PWM_BITS,
               RIGHT_MOTOR_INVERT, MOTOR_MAX_DUTY);
  Serial.printf("[motores] BTS7960 a %u Hz, duty maximo %.2f (%.1f V en el motor)\n", (unsigned)PWM_FREQ_HZ,
                MOTOR_MAX_DUTY, MOTOR_V_MAX);

  if (!encL.begin(PIN_ENC_L_A, PIN_ENC_L_B, LEFT_ENC_INVERT, ENC_FILTER_APB, ENC_INTERNAL_PULLUP))
    Serial.println("[enc] fallo PCNT izquierdo");
  if (!encR.begin(PIN_ENC_R_A, PIN_ENC_R_B, RIGHT_ENC_INVERT, ENC_FILTER_APB, ENC_INTERNAL_PULLUP))
    Serial.println("[enc] fallo PCNT derecho");

  // Calibración del giroscopio al arrancar: deja el robot quieto ~1 s
  Mpu6050::Config ic{PIN_SDA, PIN_SCL, I2C_FREQ, I2C_TIMEOUT_MS, IMU_I2C_ADDRESS, IMU_YAW_SIGN,
                     IMU_CAL_SAMPLES, IMU_CAL_MAX_STD_DPS};
  if (imu.begin(ic)) {
    const char *cal = "";
    switch (imu.bootCalibration()) {
      case Mpu6050::BootCal::OK: cal = "sesgo calibrado al arrancar"; break;
      case Mpu6050::BootCal::RESTORED: cal = "se movio al arrancar: sesgo guardado restaurado"; break;
      default: cal = "SIN calibrar (se movio): calibra desde la web"; break;
    }
    Serial.printf("[imu] MPU6050 en 0x%02X (WHO_AM_I 0x%02X), b_z = %.3f dps, %s\n", IMU_I2C_ADDRESS,
                  imu.whoAmI(), imu.biasDps(), cal);
  } else {
    Serial.printf("[imu] MPU6050 NO responde en 0x%02X (se reintenta cada 2 s)\n", IMU_I2C_ADDRESS);
  }

  motion.begin({&motorL, &motorR, &encL, &encR, &imu, &pose});
  safety.begin(&motorL, &motorR, &encL, &encR);
  mission.begin(&motion, &pose);
  mission.onPlanReady([] { planReady = true; });

  // Tareas de tiempo real en el núcleo 1
  xTaskCreatePinnedToCore(controlTask, "control", 4096, nullptr, 5, nullptr, 1);
  xTaskCreatePinnedToCore(safetyTask, "safety", 4096, nullptr, 5, nullptr, 1);
  xTaskCreatePinnedToCore(missionTask, "mission", 4096, nullptr, 3, nullptr, 1);
  // Red y planificación en el núcleo 0
  xTaskCreatePinnedToCore(plannerTask, "planner", 8192, nullptr, 3, nullptr, 0);
  delay(10);  // que la tarea planner registre su handle antes de recibir pedidos

  webapi::begin({&mission, &motion, &safety, &imu, &pose, &brush});
  xTaskCreatePinnedToCore(telemetryTask, "telemetry", 6144, nullptr, 1, nullptr, 0);

  Serial.printf("[roomba-grid-s3] listo: %s\n", webapi::networkInfo());
}

void loop() {
  // Todo corre en tareas; loop() solo cede el procesador.
  vTaskDelay(pdMS_TO_TICKS(1000));
}
