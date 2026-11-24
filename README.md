# rmb-S3-EC-6050 — Propuesta S3

Robot diferencial tipo Roomba que recorre metas en una cuadrícula de 6×6 celdas de 50 cm.
**ESP32-S3** + PlatformIO, dos puentes H **BTS7960** (uno por motor, cada uno con su EN), dos motorreductores
con **encoder de cuadratura**, **MPU6050**, una **barredora** (cepillo) en GPIO 4 y una página web (tema claro)
servida por la propia ESP32 para marcar inicio, obstáculos y metas, y elegir A\*, BFS, DFS o UCS.

Sin Raspberry Pi, sin monitor de batería y sin zumbador. Parte de las propuestas anteriores:
encoder de cuadratura, MPU6050, odometría, control, planificador, misión y web de la **Propuesta A**;
driver BTS7960 de la **Propuesta B** (ahora con un EN por lado); tema claro de la **Propuesta C**.

## Qué hay en la carpeta

```text
rmb-S3-EC-6050/
├── platformio.ini            # entornos esp32s3 (robot) y native (pruebas en la PC)
├── include/roomba_config.h   # pines, medidas del robot, ganancias y límites
├── include/roomba_settings.h # ajustes que se cambian desde la web y se guardan en NVS
├── src/main.cpp              # arranque y tareas de FreeRTOS
├── lib/
│   ├── Bts7960/              # PWM 20 kHz en RPWM/LPWM, EN por lado, freno activo, límite de voltaje
│   ├── Brush/                # barredora: automática al ejecutar o a mano desde la web
│   ├── QuadEncoder/          # cuadratura ×4 con PCNT (ESP32Encoder) y velocidad por ventana de 50 ms
│   ├── Mpu6050/              # controlador propio: DLPF 44 Hz, ±250 °/s, sesgo b_z con 500 muestras
│   ├── Pose/                 # odometría + giroscopio Z con filtro complementario (β = 0.95)
│   ├── Motion/               # PI de ruedas, avance, giro, frenado
│   ├── Safety/               # atasco (por encoder) y latido del navegador
│   ├── Planner/              # Grid, BFS, DFS, UCS, A*, Held–Karp, 2-opt, primitivas (sin Arduino)
│   ├── Mission/              # máquina de estados y cola de primitivas
│   └── WebApi/               # Wi-Fi, REST y WebSocket
├── data/                     # index.html, app.js, style.css (se suben a LittleFS)
└── test/test_planner/        # 18 pruebas Unity del planificador
```

## Pines del ESP32-S3

| Señal | GPIO | Notas |
| --- | --- | --- |
| BTS7960 izq. LPWM (atrás) | 38 | En la DevKitC-1 **v1.1** el LED RGB de la placa está en el 38: parpadeará con el PWM (inofensivo) |
| BTS7960 izq. RPWM (adelante) | 37 | ⚠ Ver "Módulos con PSRAM octal" abajo |
| BTS7960 izq. EN (R_EN + L_EN) | 39 | 10 kΩ a GND |
| BTS7960 der. LPWM (atrás) | 16 | |
| BTS7960 der. RPWM (adelante) | 15 | |
| BTS7960 der. EN (R_EN + L_EN) | 17 | 10 kΩ a GND |
| Encoder izq. A / B | 45 / 48 | ⚠ 45 es pin de arranque (ver abajo). En la DevKitC-1 **v1.0** el LED RGB está en el 48 |
| Encoder der. A / B | 10 / 11 | |
| Barredora | 4 | A un módulo MOSFET o relé, nunca al motor directo. 10 kΩ a GND |
| SDA / SCL (MPU6050) | **8 / 9** | Elegidos por mí: son los I²C por defecto del S3 y no tienen otra función. 400 kHz, dirección 0x68 |
| INT (MPU6050) | 7 | Opcional; el firmware no lo usa |
| Botón paro / inicio | **5** | A GND, INPUT_PULLUP (en la Propuesta A estaba en el 4, que ahora es la barredora) |
| LED de estado | **6** | LED externo con 330 Ω a GND (el RGB de la placa choca con 38/48) |

**No uses** en el S3: 19/20 (USB nativo), 43/44 (UART0: el monitor serie), 26–32 (flash), 0/3/46 (arranque).

### Advertencias de pines (revísalas antes de conectar)

1. **GPIO 45 (encoder izq. A) es pin de arranque**: elige el voltaje de la flash. Si está en **alto** al encender,
   la ESP32-S3 pone la flash a 1.8 V y **no arranca** (se queda muda o reinicia en bucle). Con un encoder de
   salidas push-pull, la línea A puede quedar en alto según la posición de la rueda.
   Si te pasa: mueve ese cable al **GPIO 47** y cambia `PIN_ENC_L_A = 47` en `roomba_config.h`.
   (No pongas pull-up externo en el 45; el firmware activa el interno después de arrancar.)
2. **Módulos con PSRAM octal (N8R8, N16R8, N32R8)**: dentro del módulo, los GPIO 35, 36 y 37 van a la PSRAM.
   Mira lo que dice la lata metálica del módulo. Si dice **R8**, el 37 (RPWM izq.) no es seguro:
   muévelo al **GPIO 40** y pon `PIN_L_RPWM = 40`. Con N8, N16 o R2 (PSRAM quad) no hay problema.
   El firmware no activa la PSRAM.
3. **LED RGB de la placa**: está en el 48 (DevKitC-1 v1.0) o en el 38 (v1.1). No estorba, solo puede encenderse
   con colores raros. Por eso el LED de estado es uno externo en el 6.
4. Los encoders y la MPU6050 van a **3.3 V**: el S3 no tolera 5 V en sus entradas.
5. Une todas las tierras (batería, BTS7960, buck, ESP32, barredora) en un solo punto.

## Paso a paso

Necesitas VS Code con la extensión PlatformIO, o PlatformIO por consola. Todos los comandos se escriben
**dentro de la carpeta del proyecto**.

**1. Pruebas del planificador en tu laptop** (no necesita la ESP32):

```bash
pio test -e native
```

Debe terminar con `18 Tests 0 Failures`. En Windows, `native` necesita `gcc`; si no lo tienes, córrelo desde WSL.

**2. Compilar, subir firmware y subir la página web:**

```bash
pio run -e esp32s3
pio run -e esp32s3 -t upload
pio run -e esp32s3 -t uploadfs
```

La DevKitC-1 tiene dos USB: usa el marcado **COM / UART**. Si solo tienes el puerto **USB** nativo, cambia
`-DARDUINO_USB_CDC_ON_BOOT=0` a `1` en `platformio.ini`. Si no entra en modo de carga, mantén **BOOT**,
pulsa **RST** y suelta BOOT.

**3. Ver los mensajes de arranque** (robot quieto: el primer segundo calibra el giroscopio):

```bash
pio device monitor
```

```text
[roomba-grid-s3] arrancando
[motores] BTS7960 a 20000 Hz, duty maximo 0.72 (6.0 V en el motor)
[imu] MPU6050 en 0x68 (WHO_AM_I 0x68), b_z = -0.412 dps, sesgo calibrado al arrancar
[roomba-grid-s3] listo: AP Roomba-UAMM-S3 192.168.4.1
```

**4. Abrir la interfaz:** red Wi-Fi `Roomba-UAMM-S3` (clave `roomba-grid`, cámbiala en `roomba_config.h`)
y abre `http://192.168.4.1`. En modo estación: `http://roomba-s3.local`.

## Primer arranque con motores (ruedas en el aire)

1. **Voltaje de los motores**: `MOTOR_V_MAX = 6` y `BATT_V_FULL = 8.4` (2S) dan un duty máximo de 0.72.
   Si tus motores son de 12 V con batería 3S, pon 12 y 12.6 (duty máximo 0.96).
2. *Pruebas*: PWM izq `0.4`, der `0.4`, 1000 ms → **Probar PWM**. Las dos ruedas deben ir hacia adelante.
   Si una va al revés, cambia `LEFT_MOTOR_INVERT` o `RIGHT_MOTOR_INVERT`.
3. *Cuentas izq / der* en Telemetría deben **subir** al avanzar. Si una baja, cambia `LEFT_ENC_INVERT` o
   `RIGHT_ENC_INVERT`. Si se queda en 0, revisa cable y alimentación del encoder.
4. **Cuentas por vuelta**: gira la rueda 10 vueltas a mano y divide entre 10. Ponlo en *Ajustes*
   (por defecto 1496 = 11 × 34 × 4, un ejemplo).
5. Radio de rueda (35.7 mm) y separación (235 mm) ya vienen con las medidas del robot.
6. **Giroscopio**: robot quieto → **Calibrar giroscopio** (5 s). Gira el robot a mano a la izquierda: θ debe subir.
   Si baja, pon `IMU_YAW_SIGN = -1.0f`.
7. **Barredora**: **Probar barredora** la enciende y apaga a mano (solo sin misión en curso).
   Si se enciende al revés (relé activo en bajo), pon `BRUSH_ACTIVE_HIGH = false`.
8. Robot en el piso: marca la celda de Inicio con su orientación → **Colocar robot en el inicio marcado** →
   `AVANZAR(1)` y `GIRAR(+90°)`. Luego metas, algoritmo, **Planear** y **Ejecutar**.

### Barredora

- **Automática** (por defecto, *Ajustes → Barredora automática al ejecutar*): gira mientras la misión está
  EJECUTANDO y se apaga en pausa, paro, falla o al terminar.
- **A mano**: botón **Probar barredora** o `POST /api/barredora {"encendida":true}`. Se apaga sola cuando
  cambia el estado de la misión.

El botón físico (GPIO 5) hace: paro si está en marcha, ejecutar si hay un plan listo, reanudar si está pausado.
La barra espaciadora en la página también manda PARO.

## Qué cambió respecto a la Propuesta A

| Tema | Propuesta A | Propuesta S3 (este código) |
| --- | --- | --- |
| Placa | ESP32 DevKit V1 | ESP32-S3 DevKitC-1 (`board = esp32-s3-devkitc-1`) |
| Puente H | L298N, PWM 2 kHz, sentido con IN1–IN4 | 2× BTS7960: PWM 20 kHz en RPWM/LPWM, un EN por lado. Freno activo = EN alto con RPWM = LPWM = 0; rueda libre = EN bajo |
| Duty máximo | 0.94 (caída de 2 V del L298N) | 0.72 = 6 V / (8.4 V − 0.1 V) |
| PWM mínimo | 0.25 | 0.15 (el BTS7960 casi no pierde voltaje; ajústalo con la prueba de PWM) |
| Encoders | 34/35 y 18/19 con pull-up externo | 45/48 y 10/11 con pull-up interno |
| I²C | 21/22 | 8/9 |
| Botón / LED | 4 / 2 | 5 / 6 |
| Barredora | — | GPIO 4, automática o a mano |
| Geometría | r 32.5 mm, B 0.20 m (provisional) | r 35.7 mm, B 0.235 m (medidas del robot) |
| Wi-Fi | `Roomba-UAMM`, `roomba.local` | `Roomba-UAMM-S3`, `roomba-s3.local` |
| Web | Tema oscuro/claro | Tema claro fijo (de la Propuesta C) |

## API

Todas las rutas POST esperan JSON. Si no llevan datos, manda `{}`.

| Ruta | Uso |
| --- | --- |
| GET /api/estado | Pose, celda, estado, velocidades, cuentas, barredora, sesgo y ω del giroscopio |
| GET, POST /api/mapa | Obstáculos `{obstaculos:[[f,c],…]}` (se guardan en NVS) |
| GET, POST /api/plan | `{algoritmo, heuristica?, inicio, orientacion, metas, regresar, obstaculos?}` |
| POST /api/ejecutar | `{confirmar?: true}` |
| POST /api/pausa | Pausa al terminar la primitiva en curso, o reanuda |
| POST /api/paro | Freno activo y luego rueda libre |
| POST /api/barredora | `{encendida:true|false}`, solo sin misión en curso |
| POST /api/calibrar | `{tipo:"imu"}`: sesgo del giroscopio con 500 muestras en reposo |
| POST /api/imu/cero | `{celda:[f,c], orientacion:"E"}` |
| GET, POST /api/config | Ganancias, velocidades, c_g, β, `barredora_auto`; `{restablecer:true}` vuelve a `roomba_config.h` |
| POST /api/prueba | `{tipo:"avanzar",celdas:1}`, `{tipo:"girar",grados:90}` o `{tipo:"pwm",izq:0.4,der:0.4,ms:1000}` |
| POST /api/wifi | `{modo:"sta",ssid:"…",clave:"…"}` o `{modo:"ap"}`; reinicia |
| WS /ws | Telemetría cada 100 ms, plan, avisos. El navegador manda `{"tipo":"latido"}` cada segundo |

## Qué está probado y qué no

- **Planificador**: 18/18 pruebas pasan (g++).
- **Firmware**: no se pudo compilar para ESP32-S3 donde se escribió (el registro de PlatformIO está bloqueado
  en esa red). `main.cpp`, `Motion` y `Mission` se revisaron con g++ contra cabeceras de prueba, sin errores
  ni avisos. **Primer paso: `pio run -e esp32s3`** y mandar cualquier error tal cual.
- **En el robot real**: nada aún.
