#pragma once
// Wi-Fi (punto de acceso o estación), servidor HTTP con la API REST y WebSocket /ws.
#include <Arduino.h>
#include "Mission.h"
#include "Motion.h"
#include "Safety.h"
#include "Mpu6050.h"
#include "Pose.h"
#include "Brush.h"

namespace webapi {

struct Context {
  Mission *mission;
  Motion *motion;
  Safety *safety;
  Mpu6050 *imu;
  Pose *pose;
  Brush *brush;
};

void begin(const Context &ctx);       // Wi-Fi + mDNS + LittleFS + servidor
void sendTelemetry();                 // tarea de telemetría (cada 100 ms)
void broadcastPlan();                 // al terminar de planear
void sendEvent(const char *text);     // aviso para la interfaz
const char *networkInfo();            // "AP Roomba-UAMM-S3 192.168.4.1" o "STA red 10.0.0.5"

}  // namespace webapi
