#pragma once
// ============================================================================
//  Roomba cuadrícula — Propuesta S3
//  ESP32-S3 DevKitC-1 + 2 × BTS7960 (IBT-2) + 2 motorreductores con encoder de cuadratura
//  + MPU6050 + barredora (cepillo) en GPIO 4.
//  Pines, constantes físicas y valores por defecto.
//  Lo marcado con (NVS) se puede cambiar desde la interfaz web sin recompilar.
//  Sin monitor de batería ni zumbador.
//
//  Mapa de pines (ver README, sección "Pines del ESP32-S3"):
//    enc izq A/B 45/48 · enc der A/B 10/11
//    BTS izq LPWM 38 RPWM 40 EN 39 · BTS der LPWM 16 RPWM 15 EN 17
//    barredora 4 · I2C SDA 8 SCL 9 · botón 5 · LED de estado 6
//  Prohibidos en el S3: 19/20 (USB), 43/44 (UART0, monitor serie), 26–32 (flash), 33–37 (PSRAM octal N16R8),
//  0/3/46 (arranque). 45 también es de arranque: ver la advertencia del encoder izquierdo.
// ============================================================================
#include <stdint.h>

// ---------------------------------------------------------------- Motores (BTS7960 / IBT-2)
// Un EN por lado: une R_EN y L_EN de cada módulo y pon 10 kΩ de EN a GND.
// RPWM = adelante, LPWM = atrás (si un motor va al revés cambia su INVERT, no los cables).
constexpr int PIN_L_LPWM = 38;  // ¡en la DevKitC-1 v1.1 el LED RGB de la placa está en el 38!
constexpr int PIN_L_RPWM = 40;  // era 37: en el módulo N16R8 los GPIO 35–37 son de la PSRAM octal
constexpr int PIN_L_EN = 39;
constexpr int PIN_R_LPWM = 16;
constexpr int PIN_R_RPWM = 15;
constexpr int PIN_R_EN = 17;

constexpr uint32_t PWM_FREQ_HZ = 20000;  // 20 kHz: inaudible, el BTS7960 (MOSFET) lo soporta
constexpr uint8_t PWM_BITS = 10;         // 0..1023 (a 20 kHz el LEDC del S3 da hasta 11 bits)
constexpr uint8_t LEDC_CH_L_R = 0, LEDC_CH_L_L = 1, LEDC_CH_R_R = 2, LEDC_CH_R_L = 3;

// Límite de voltaje en el motor. El BTS7960 casi no tiene caída (~0.1 V), así que con una
// batería 2S recién cargada (8.4 V) y motores de 6 V: duty máx = 6 / (8.4 − 0.1) ≈ 0.72.
// Si tus motores son de 12 V con batería 3S, pon MOTOR_V_MAX = 12 y BATT_V_FULL = 12.6.
constexpr float MOTOR_V_MAX = 6.0f;
constexpr float BATT_V_FULL = 8.4f;
constexpr float BTS7960_DROP_V = 0.1f;
constexpr float MOTOR_MAX_DUTY = MOTOR_V_MAX / (BATT_V_FULL - BTS7960_DROP_V) > 1.0f
                                     ? 1.0f
                                     : MOTOR_V_MAX / (BATT_V_FULL - BTS7960_DROP_V);

constexpr bool LEFT_MOTOR_INVERT = false;
constexpr bool RIGHT_MOTOR_INVERT = false;

// ---------------------------------------------------------------- Encoders de cuadratura
// Alimenta los encoders a 3.3 V: el S3 NO tolera 5 V en sus entradas.
// GPIO 45 es pin de arranque (voltaje de la flash): si está en ALTO al encender, la placa no
// arranca. Si el encoder izquierdo deja la línea A en alto al encender (con la rueda en cierta
// posición), la ESP32-S3 se queda muda: cambia PIN_ENC_L_A a 47 (o 12/13/14/21) y mueve el cable.
constexpr int PIN_ENC_L_A = 45;
constexpr int PIN_ENC_L_B = 48;  // en la DevKitC-1 v1.0 el LED RGB está en el 48 (inofensivo como entrada)
constexpr int PIN_ENC_R_A = 10;
constexpr int PIN_ENC_R_B = 11;
// Pull-up interno del S3 (~45 kΩ). Con cables largos o encoders de colector abierto añade 4.7 kΩ a 3.3 V.
constexpr bool ENC_INTERNAL_PULLUP = true;
// Si al avanzar una cuenta baja (se ve en Telemetría), cambia su INVERT o intercambia A y B.
constexpr bool LEFT_ENC_INVERT = false;
constexpr bool RIGHT_ENC_INVERT = false;
// Ejemplo: 11 pulsos/vuelta del motor × reducción 34 × 4 flancos = 1496.
// Confírmalo con la hoja de datos de tu motor y con 10 vueltas marcadas a mano.
constexpr float CUENTAS_POR_VUELTA = 1496.0f;  // (NVS)
constexpr uint16_t ENC_FILTER_APB = 500;       // filtro PCNT: 500 ciclos de 80 MHz = 6.25 µs
constexpr uint32_t ENC_STILL_MS = 100;         // sin cuentas en 100 ms → rueda quieta

// ---------------------------------------------------------------- Geometría (medida en el robot)
constexpr float WHEEL_RADIUS_M = 0.0357f;  // (NVS) radio de la rueda, 35.7 mm
constexpr float WHEEL_BASE_M = 0.235f;     // (NVS) separación entre ruedas, 235 mm
constexpr float CELL_SIZE_M = 0.50f;

// ---------------------------------------------------------------- MPU6050 (GY-521)
// GPIO 8 y 9 son los pines I2C por defecto del S3 en Arduino y no tienen otra función.
// El GY-521 ya trae pull-up de 4.7 kΩ en SDA/SCL. Aliméntalo por VCC (3.3 V o 5 V, tiene regulador).
constexpr int PIN_SDA = 8;
constexpr int PIN_SCL = 9;
constexpr int PIN_MPU_INT = 7;            // dato listo; opcional, no se usa por ahora
constexpr uint32_t I2C_FREQ = 400000;     // 400 kHz
constexpr uint16_t I2C_TIMEOUT_MS = 10;
constexpr uint8_t IMU_I2C_ADDRESS = 0x68; // AD0 en bajo (0x69 si AD0 en alto)
// +1 si el chip está con el eje Z hacia arriba (componentes arriba). Si al girar a la
// izquierda el rumbo baja, pon −1.
constexpr float IMU_YAW_SIGN = 1.0f;
constexpr uint16_t IMU_CAL_SAMPLES = 500;   // muestras para el sesgo b_z
constexpr float IMU_CAL_MAX_STD_DPS = 0.5f; // más ruido que esto = el robot se movió
constexpr float GYRO_BETA = 0.95f;          // (NVS) peso del giroscopio en el filtro complementario

// ---------------------------------------------------------------- Barredora (cepillo)
// GPIO 4 → módulo MOSFET o relé (nunca el motor directo al pin: da 40 mA como máximo).
// Pon 10 kΩ de GPIO 4 a GND para que no arranque sola mientras la ESP32 inicia.
constexpr int PIN_BRUSH = 4;
constexpr bool BRUSH_ACTIVE_HIGH = true;  // false si tu módulo de relé se activa en bajo
// En automático gira solo mientras la misión está EJECUTANDO; se apaga en pausa, paro o falla.

// ---------------------------------------------------------------- Interfaz física
constexpr int PIN_BUTTON = 5;  // paro / inicio, a GND, INPUT_PULLUP (el 4 ahora es la barredora)
constexpr int PIN_LED = 6;     // LED de estado externo con 330 Ω a GND (el RGB de la placa choca con 38/48)

// ---------------------------------------------------------------- Valores por defecto (NVS)
struct DefaultGains {
  // Velocidad por rueda: duty = signo·(PWM_MIN + KFF·|v|) + Kp·e + Ki·∫e
  // Punto de partida para un JGA25-370 de 6 V con BTS7960; ajústalo con la prueba de PWM de la web.
  static constexpr float WHEEL_KP = 1.0f;    // duty por (m/s)
  static constexpr float WHEEL_KI = 5.0f;    // duty por (m/s·s)
  static constexpr float WHEEL_KFF = 1.5f;   // duty por (m/s)
  static constexpr float WHEEL_PWM_MIN = 0.15f;  // arranque del motor; el BTS7960 casi no pierde voltaje
  // Avance: ω_ref = Kθ·eθ + Ky·ey
  static constexpr float HEADING_K = 4.0f;   // (rad/s)/rad
  static constexpr float LATERAL_K = 6.0f;   // (rad/s)/m
  // Giro en sitio: PD sobre el rumbo
  static constexpr float TURN_KP = 5.0f;     // (rad/s)/rad
  static constexpr float TURN_KD = 0.25f;
};

constexpr float V_MAX = 0.25f;            // m/s (NVS)
constexpr float ACCEL = 0.50f;            // m/s² (NVS)
constexpr float V_APPROACH = 0.08f;       // m/s, últimos 10 cm (NVS)
constexpr float APPROACH_DIST = 0.10f;    // m
constexpr float TURN_W_MAX_DEG = 90.0f;   // °/s (NVS)
constexpr float TURN_ALPHA_DEG = 180.0f;  // °/s² (NVS)
constexpr float TURN_W_MIN_DEG = 12.0f;   // °/s mínimos mientras falte giro
constexpr float TURN_TOL_DEG = 1.5f;      // |eθ| < 1.5°
constexpr float TURN_STILL_DEG_S = 5.0f;  // |ω| < 5 °/s
constexpr uint16_t TURN_SETTLE_MS = 100;  // ...durante 100 ms
constexpr float FORWARD_STOP_M = 0.005f;  // distancia restante < 5 mm
constexpr uint16_t PRIMITIVE_PAUSE_MS = 150;  // pausa con freno entre primitivas
constexpr float SLIP_LIMIT_DEG = 3.0f;    // encoder vs giroscopio durante un avance
constexpr float TURN_COST = 0.5f;         // c_g del planificador (NVS)

// Atasco: PWM alto y sin cuentas del encoder durante STALL_TIME_MS
constexpr float STALL_MIN_DUTY = 0.35f;
constexpr uint16_t STALL_TIME_MS = 500;
constexpr uint16_t HEARTBEAT_TIMEOUT_MS = 2000;  // latido del navegador (NVS: activar/desactivar)

// ---------------------------------------------------------------- Wi-Fi
constexpr const char *AP_SSID = "Roomba-UAMM-S3";
constexpr const char *AP_PASSWORD = "roomba-grid";  // WPA2: mínimo 8 caracteres. ¡Cámbiala!
constexpr const char *MDNS_NAME = "roomba-s3";      // roomba-s3.local en modo estación
constexpr uint32_t STA_CONNECT_TIMEOUT_MS = 15000;

// ---------------------------------------------------------------- Tareas (FreeRTOS)
constexpr uint32_t CONTROL_PERIOD_MS = 10;    // 100 Hz, núcleo 1
constexpr uint32_t SAFETY_PERIOD_MS = 20;     // núcleo 1
constexpr uint32_t MISSION_PERIOD_MS = 50;    // núcleo 1
constexpr uint32_t TELEMETRY_PERIOD_MS = 100; // núcleo 0
