// Prueba de la placa sola, sin el firmware del robot:  pio run -e diag -t upload && pio device monitor
// 1) 5 s solo imprimiendo (sin tocar pines ni Wi-Fi)
// 2) pone cada pin de salida en bajo, uno por segundo
// 3) enciende el Wi-Fi (punto de acceso)
// Si se reinicia, la última línea dice en qué paso fue.
#include <Arduino.h>
#include <WiFi.h>

static const int PINS[] = {39, 17, 40, 38, 15, 16, 4};

static void paso(const char *m) {
  Serial.printf("[diag %6lu ms] %s\n", (unsigned long)millis(), m);
  Serial.flush();
}

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.printf("\n[diag] inicio, causa de reinicio = %d\n", (int)esp_reset_reason());
  for (int i = 0; i < 10; i++) {
    paso("vivo, sin tocar pines");
    delay(500);
  }
  char buf[48];
  for (int p : PINS) {
    snprintf(buf, sizeof(buf), "GPIO %d como salida en bajo", p);
    paso(buf);
    pinMode(p, OUTPUT);
    digitalWrite(p, LOW);
    delay(1000);
  }
  paso("encendiendo Wi-Fi");
  WiFi.softAP("Roomba-diag", "roomba-grid");
  paso("Wi-Fi encendido");
}

void loop() {
  paso("todo bien");
  delay(2000);
}
