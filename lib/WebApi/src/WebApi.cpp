#include "WebApi.h"

#include <ArduinoJson.h>
#include <AsyncJson.h>
#include <ESPAsyncWebServer.h>
#include <ESPmDNS.h>
#include <LittleFS.h>
#include <Preferences.h>
#include <WiFi.h>

#include "roomba_config.h"
#include "roomba_settings.h"

using namespace planner;

namespace webapi {

namespace {

Context C;
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");
char netInfo[64] = "";

// ---------------------------------------------------------------- utilidades JSON
void sendJson(AsyncWebServerRequest *req, int code, const JsonDocument &doc) {
  String out;
  serializeJson(doc, out);
  req->send(code, "application/json", out);
}

void sendOk(AsyncWebServerRequest *req, const char *msg = "ok") {
  JsonDocument d;
  d["ok"] = true;
  d["mensaje"] = msg;
  sendJson(req, 200, d);
}

void sendError(AsyncWebServerRequest *req, int code, const char *msg) {
  JsonDocument d;
  d["ok"] = false;
  d["error"] = msg;
  sendJson(req, code, d);
}

bool readCell(JsonVariantConst v, Cell &c) {
  if (!v.is<JsonArrayConst>() || v.size() != 2) return false;
  c = Cell(v[0].as<int>(), v[1].as<int>());
  return inBounds(c);
}

void addCell(JsonArray a, Cell c) {
  JsonArray x = a.add<JsonArray>();
  x.add(c.f);
  x.add(c.c);
}

// ---------------------------------------------------------------- configuración
struct FloatField {
  const char *name;
  float Settings::*ptr;
  float lo, hi;
};

const FloatField FIELDS[] = {
    {"cuentas_por_vuelta", &Settings::ticksPerRev, 10, 5000},
    {"radio_rueda", &Settings::wheelRadius, 0.01f, 0.2f},
    {"separacion_ruedas", &Settings::wheelBase, 0.05f, 0.5f},
    {"rueda_kp", &Settings::wheelKp, 0, 20},
    {"rueda_ki", &Settings::wheelKi, 0, 100},
    {"rueda_kff", &Settings::wheelKff, 0, 20},
    {"pwm_min", &Settings::pwmMin, 0, 0.8f},
    {"rumbo_k", &Settings::headingK, 0, 50},
    {"lateral_k", &Settings::lateralK, 0, 50},
    {"giro_kp", &Settings::turnKp, 0, 50},
    {"giro_kd", &Settings::turnKd, 0, 10},
    {"v_max", &Settings::vMax, 0.05f, 0.6f},
    {"aceleracion", &Settings::accel, 0.05f, 3},
    {"v_aproximacion", &Settings::vApproach, 0.02f, 0.3f},
    {"giro_w_max", &Settings::turnWMaxDeg, 10, 360},
    {"giro_alfa", &Settings::turnAlphaDeg, 10, 1000},
    {"costo_giro", &Settings::turnCost, 0, 10},
    {"beta_giro", &Settings::gyroBeta, 0, 1},
};

void configToJson(JsonDocument &d) {
  for (const auto &f : FIELDS) d[f.name] = gSettings.*(f.ptr);
  d["paro_sin_latido"] = gSettings.heartbeatStop;
  d["barredora_auto"] = gSettings.brushAuto;
}

// ---------------------------------------------------------------- estado y plan
void stateToJson(JsonDocument &d) {
  Mission &m = *C.mission;
  Motion::Telemetry t = C.motion->telemetry();
  Pose &p = *C.pose;
  d["x"] = p.x;
  d["y"] = p.y;
  d["theta"] = p.theta * RAD_TO_DEG;
  Cell c = m.currentCell();
  JsonArray cel = d["celda"].to<JsonArray>();
  cel.add(c.f);
  cel.add(c.c);
  d["estado"] = Mission::stateName(m.state());
  d["mensaje"] = m.message();
  d["movimiento"] = Motion::stateName(t.state);
  char label[24];
  m.currentLabel(label, sizeof(label));
  d["primitiva"] = label;
  d["progreso"] = t.progress;
  d["paso"] = m.stepIndex();
  d["pasos"] = m.stepCount();
  d["metas_alcanzadas"] = m.goalsReached();
  d["v_izq"] = t.vL;
  d["v_der"] = t.vR;
  d["pwm_izq"] = t.dutyL;
  d["pwm_der"] = t.dutyR;
  d["motores_activos"] = C.motion->motorsEnabled();
  d["barredora"] = C.brush->on();
  JsonArray tk = d["cuentas"].to<JsonArray>();
  tk.add((long)t.ticksL);
  tk.add((long)t.ticksR);
  d["deslizamientos"] = t.slipEvents;
  d["dist_frenado_mm"] = t.brakeDist * 1000.0f;
  d["odometro"] = p.distance;
  JsonObject cal = d["calib"].to<JsonObject>();
  cal["sesgo_dps"] = C.imu->biasDps();
  cal["calibrado"] = C.imu->calibrated();
  cal["calibrando"] = C.imu->calibrating();
  cal["resultado"] = C.imu->calibrationResult();  // 1 bien, -1 se movio, 0 sin resultado
  cal["desv_dps"] = C.imu->lastCalibrationStd();
  const char *arr = "sin calibrar";
  if (C.imu->bootCalibration() == Mpu6050::BootCal::OK) arr = "calibrado";
  else if (C.imu->bootCalibration() == Mpu6050::BootCal::RESTORED) arr = "restaurado";
  cal["arranque"] = arr;
  d["imu_ok"] = t.imuOk;
  d["imu_reinicios"] = C.imu->resets();
  d["omega"] = C.imu->yawRate() * RAD_TO_DEG;
  d["red"] = netInfo;
  d["t"] = millis();
}

void planToJson(JsonDocument &d) {
  C.mission->withPlan([&](bool valid, const Mission::PlanRequest &req, const TourResult &t) {
    d["tipo"] = "plan";
    d["ok"] = valid;
    if (t.error) d["error"] = t.error;
    d["algoritmo"] = algorithmName(req.algo);
    d["heuristica"] = req.heuristic == Heuristic::MANHATTAN_TURNS ? "manhattan_giros" : "manhattan";
    JsonArray ini = d["inicio"].to<JsonArray>();
    ini.add(req.start.f);
    ini.add(req.start.c);
    char o[2] = {headingChar(req.heading), 0};
    d["orientacion"] = o;
    d["regresar"] = req.returnHome;
    JsonArray metas = d["metas"].to<JsonArray>();
    for (const Cell &c : req.goals) addCell(metas, c);
    JsonArray orden = d["orden"].to<JsonArray>();
    for (uint8_t i : t.order) orden.add(i);
    JsonArray inal = d["inalcanzables"].to<JsonArray>();
    for (uint8_t i : t.unreachable) inal.add(i);
    d["exacto"] = t.exact;
    d["costo"] = t.cost;
    d["celdas"] = t.cells;
    d["giros"] = t.turns;
    d["expandidos"] = t.expanded;
    d["expandidos_matriz"] = t.expandedMatrix;
    d["tiempo_us"] = t.timeUs;
    JsonArray rutas = d["rutas"].to<JsonArray>();
    JsonArray exp = d["expandidos_orden"].to<JsonArray>();
    for (const Leg &leg : t.legs) {
      JsonArray r = rutas.add<JsonArray>();
      for (const Cell &c : leg.res.path) addCell(r, c);
      JsonArray e = exp.add<JsonArray>();
      for (uint8_t i : leg.res.expanded) e.add(i);
    }
    JsonArray prim = d["primitivas"].to<JsonArray>();
    if (valid) {
      std::vector<Primitive> ps = tourToPrimitives(t, req.start, req.heading);
      for (const Primitive &p : ps) {
        char b[24];
        if (p.type == PrimType::FORWARD) snprintf(b, sizeof(b), "AVANZAR(%u)", p.cells);
        else snprintf(b, sizeof(b), "GIRAR(%+d)", p.turnDeg);
        prim.add(b);
      }
    }
  });
}

// ---------------------------------------------------------------- WebSocket
void onWsEvent(AsyncWebSocket *, AsyncWebSocketClient *client, AwsEventType type, void *arg,
               uint8_t *data, size_t len) {
  if (type == WS_EVT_CONNECT) {
    C.safety->heartbeat();
    JsonDocument d;
    planToJson(d);
    String out;
    serializeJson(d, out);
    client->text(out);
  } else if (type == WS_EVT_DISCONNECT) {
    if (ws.count() == 0) C.safety->forgetHeartbeat();
  } else if (type == WS_EVT_DATA) {
    AwsFrameInfo *info = (AwsFrameInfo *)arg;
    if (!(info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT)) return;
    JsonDocument d;
    if (deserializeJson(d, data, len)) return;
    const char *tipo = d["tipo"] | "";
    if (!strcmp(tipo, "latido")) C.safety->heartbeat();
    else if (!strcmp(tipo, "paro")) C.mission->emergencyStop();  // paro también por WS: más rápido
  }
}

// ---------------------------------------------------------------- Wi-Fi
void startWifi() {
  Preferences p;
  String mode = "ap", ssid, pass;
  if (p.begin("wifi", true)) {
    mode = p.getString("modo", "ap");
    ssid = p.getString("ssid", "");
    pass = p.getString("clave", "");
    p.end();
  }
  if (mode == "sta" && ssid.length()) {
    WiFi.mode(WIFI_STA);
    WiFi.setHostname(MDNS_NAME);
    WiFi.begin(ssid.c_str(), pass.c_str());
    uint32_t t0 = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t0 < STA_CONNECT_TIMEOUT_MS) delay(200);
    if (WiFi.status() == WL_CONNECTED) {
      WiFi.setSleep(false);
      MDNS.begin(MDNS_NAME);
      MDNS.addService("http", "tcp", 80);
      snprintf(netInfo, sizeof(netInfo), "STA %s %s (%s.local)", ssid.c_str(),
               WiFi.localIP().toString().c_str(), MDNS_NAME);
      return;
    }
    WiFi.disconnect(true);
  }
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASSWORD);
  WiFi.setSleep(false);
  snprintf(netInfo, sizeof(netInfo), "AP %s %s", AP_SSID, WiFi.softAPIP().toString().c_str());
}

// ---------------------------------------------------------------- rutas
void addJsonRoute(const char *path, std::function<void(AsyncWebServerRequest *, JsonVariant &)> fn) {
  auto *h = new AsyncCallbackJsonWebHandler(path, fn);
  h->setMethod(HTTP_POST);
  h->setMaxContentLength(4096);
  server.addHandler(h);
}

void setupRoutes() {
  server.on("/api/estado", HTTP_GET, [](AsyncWebServerRequest *req) {
    JsonDocument d;
    stateToJson(d);
    sendJson(req, 200, d);
  });

  server.on("/api/mapa", HTTP_GET, [](AsyncWebServerRequest *req) {
    JsonDocument d;
    JsonArray obs = d["obstaculos"].to<JsonArray>();
    Grid g = C.mission->grid();
    for (int i = 0; i < NCELLS; i++)
      if (g.isBlocked(cellFromIndex(i))) addCell(obs, cellFromIndex(i));
    sendJson(req, 200, d);
  });

  addJsonRoute("/api/mapa", [](AsyncWebServerRequest *req, JsonVariant &json) {
    std::vector<Cell> obs;
    for (JsonVariantConst v : json["obstaculos"].as<JsonArrayConst>()) {
      Cell c;
      if (!readCell(v, c)) return sendError(req, 400, "celda fuera de la cuadricula");
      obs.push_back(c);
    }
    Mission::State s = C.mission->state();
    if (s == Mission::State::EJECUTANDO || s == Mission::State::PAUSADO || s == Mission::State::PLANEANDO)
      return sendError(req, 409, "no se puede cambiar el mapa durante una mision");
    C.mission->setObstacles(obs);
    sendOk(req, "mapa guardado");
  });

  server.on("/api/plan", HTTP_GET, [](AsyncWebServerRequest *req) {
    JsonDocument d;
    planToJson(d);
    sendJson(req, 200, d);
  });

  addJsonRoute("/api/plan", [](AsyncWebServerRequest *req, JsonVariant &json) {
    Mission::PlanRequest r;
    const char *alg = json["algoritmo"] | "astar";
    if (!parseAlgorithm(alg, r.algo)) return sendError(req, 400, "algoritmo: astar, bfs, dfs o ucs");
    const char *heur = json["heuristica"] | "manhattan";
    r.heuristic = !strcmp(heur, "manhattan_giros") ? Heuristic::MANHATTAN_TURNS : Heuristic::MANHATTAN;
    if (!readCell(json["inicio"], r.start)) return sendError(req, 400, "inicio invalido");
    const char *o = json["orientacion"] | "E";
    if (!parseHeading(o[0], r.heading)) return sendError(req, 400, "orientacion: N, E, S u O");
    for (JsonVariantConst v : json["metas"].as<JsonArrayConst>()) {
      Cell c;
      if (!readCell(v, c)) return sendError(req, 400, "meta fuera de la cuadricula");
      bool dup = false;
      for (const Cell &k : r.goals) dup |= (k == c);
      if (!dup) r.goals.push_back(c);
    }
    r.returnHome = json["regresar"] | false;
    // Si vienen obstáculos en el mismo cuerpo, se guardan primero
    if (json["obstaculos"].is<JsonArrayConst>()) {
      std::vector<Cell> obs;
      for (JsonVariantConst v : json["obstaculos"].as<JsonArrayConst>()) {
        Cell c;
        if (readCell(v, c)) obs.push_back(c);
      }
      Mission::State s = C.mission->state();
      if (s != Mission::State::EJECUTANDO && s != Mission::State::PAUSADO && s != Mission::State::PLANEANDO)
        C.mission->setObstacles(obs);
    }
    const char *err = "";
    if (!C.mission->requestPlan(r, &err)) return sendError(req, 409, err);
    JsonDocument d;
    d["ok"] = true;
    d["mensaje"] = "planeando; el resultado llega por /ws o GET /api/plan";
    sendJson(req, 202, d);
  });

  addJsonRoute("/api/ejecutar", [](AsyncWebServerRequest *req, JsonVariant &json) {
    const char *err = "";
    bool confirm = json["confirmar"] | false;
    if (!C.mission->execute(confirm, &err)) return sendError(req, 409, err);
    sendOk(req, "ejecutando");
  });

  addJsonRoute("/api/pausa", [](AsyncWebServerRequest *req, JsonVariant &) {
    const char *err = "";
    if (!C.mission->togglePause(&err)) return sendError(req, 409, err);
    sendOk(req, C.mission->message());
  });

  // El paro acepta cualquier POST, con o sin cuerpo JSON
  server.on("/api/paro", HTTP_POST, [](AsyncWebServerRequest *req) {
    C.mission->emergencyStop();
    sendOk(req, "paro");
  });

  // Calibración del sesgo del giroscopio: 500 muestras con el robot quieto (5 s).
  // También en /api/calibrar con {"tipo":"imu"}, como en la propuesta.
  auto calibrar = [](AsyncWebServerRequest *req, JsonVariant &json) {
    const char *tipo = json["tipo"] | "imu";
    if (strcmp(tipo, "imu")) return sendError(req, 400, "tipo: imu");
    Mission::State s = C.mission->state();
    if (s == Mission::State::EJECUTANDO || s == Mission::State::PAUSADO)
      return sendError(req, 409, "no durante una mision");
    if (!C.imu->ok()) return sendError(req, 503, "la MPU6050 no responde");
    C.imu->startCalibration();
    sendOk(req, "no muevas el robot durante 5 s; el sesgo se guarda solo");
  };
  addJsonRoute("/api/imu/calibrar", calibrar);
  addJsonRoute("/api/calibrar", calibrar);

  addJsonRoute("/api/imu/cero", [](AsyncWebServerRequest *req, JsonVariant &json) {
    Mission::State s = C.mission->state();
    if (s == Mission::State::EJECUTANDO || s == Mission::State::PAUSADO)
      return sendError(req, 409, "no durante una mision");
    // Coloca el robot: celda y orientación actuales (por defecto, la celda estimada mirando al E)
    Cell c = C.mission->currentCell();
    if (!inBounds(c)) c = Cell(0, 0);
    readCell(json["celda"], c);
    uint8_t h = E;
    const char *o = json["orientacion"] | "E";
    parseHeading(o[0], h);
    C.mission->setPoseAt(c, h);
    sendOk(req, "rumbo y pose fijados");
  });

  server.on("/api/config", HTTP_GET, [](AsyncWebServerRequest *req) {
    JsonDocument d;
    configToJson(d);
    sendJson(req, 200, d);
  });

  addJsonRoute("/api/config", [](AsyncWebServerRequest *req, JsonVariant &json) {
    Mission::State s = C.mission->state();
    if (s == Mission::State::EJECUTANDO || s == Mission::State::PAUSADO)
      return sendError(req, 409, "no durante una mision");
    if (json["restablecer"] | false) {
      gSettings.reset();
    } else {
      Settings n = gSettings;
      for (const auto &f : FIELDS) {
        JsonVariantConst v = json[f.name];
        if (v.isNull()) continue;
        float x = v.as<float>();
        if (isnan(x) || x < f.lo || x > f.hi) {
          static char msg[64];
          snprintf(msg, sizeof(msg), "%s fuera de rango (%.3g a %.3g)", f.name, f.lo, f.hi);
          return sendError(req, 400, msg);
        }
        n.*(f.ptr) = x;
      }
      if (!json["paro_sin_latido"].isNull()) n.heartbeatStop = json["paro_sin_latido"].as<bool>();
      if (!json["barredora_auto"].isNull()) n.brushAuto = json["barredora_auto"].as<bool>();
      gSettings = n;
    }
    gSettings.save();
    JsonDocument d;
    configToJson(d);
    d["ok"] = true;
    sendJson(req, 200, d);
  });

  addJsonRoute("/api/prueba", [](AsyncWebServerRequest *req, JsonVariant &json) {
    const char *tipo = json["tipo"] | "";
    const char *err = "tipo: avanzar, girar o pwm";
    bool ok = false;
    if (!strcmp(tipo, "avanzar")) ok = C.mission->testForward(json["celdas"] | 1, &err);
    else if (!strcmp(tipo, "girar")) ok = C.mission->testTurn(json["grados"] | 90, &err);
    else if (!strcmp(tipo, "pwm"))
      ok = C.mission->testPwm(json["izq"] | 0.0f, json["der"] | 0.0f, json["ms"] | 1000, &err);
    if (!ok) return sendError(req, 409, err);
    sendOk(req, "prueba en curso");
  });

  // Barredora a mano (prueba): {"encendida": true|false}. No durante una misión.
  addJsonRoute("/api/barredora", [](AsyncWebServerRequest *req, JsonVariant &json) {
    Mission::State s = C.mission->state();
    bool on = json["encendida"] | false;
    if (on && (s == Mission::State::EJECUTANDO || s == Mission::State::PAUSADO))
      return sendError(req, 409, "durante una mision la barredora va en automatico");
    C.brush->setManual(on);
    sendOk(req, on ? "barredora encendida" : "barredora apagada");
  });

  addJsonRoute("/api/wifi", [](AsyncWebServerRequest *req, JsonVariant &json) {
    const char *modo = json["modo"] | "ap";
    Preferences p;
    if (!p.begin("wifi", false)) return sendError(req, 500, "NVS");
    p.putString("modo", !strcmp(modo, "sta") ? "sta" : "ap");
    p.putString("ssid", json["ssid"] | "");
    p.putString("clave", json["clave"] | "");
    p.end();
    sendOk(req, "guardado; reiniciando en 1 s");
    // Reinicio diferido para que la respuesta alcance a salir
    xTaskCreate([](void *) {
      vTaskDelay(pdMS_TO_TICKS(1000));
      ESP.restart();
    }, "reboot", 2048, nullptr, 1, nullptr);
  });

  ws.onEvent(onWsEvent);
  server.addHandler(&ws);

  // Interfaz: archivos de LittleFS (sirve index.html.gz si existe)
  server.serveStatic("/", LittleFS, "/").setDefaultFile("index.html").setCacheControl("no-cache");
  server.onNotFound([](AsyncWebServerRequest *req) {
    if (req->method() == HTTP_OPTIONS) return req->send(204);
    req->send(404, "text/plain", "no encontrado (¿subiste la interfaz con pio run -t uploadfs?)");
  });
  DefaultHeaders::Instance().addHeader("Access-Control-Allow-Origin", "*");
  DefaultHeaders::Instance().addHeader("Access-Control-Allow-Headers", "Content-Type");
}

}  // namespace

void begin(const Context &ctx) {
  C = ctx;
  if (!LittleFS.begin(true)) Serial.println("[web] LittleFS no monto");
  startWifi();
  Serial.printf("[web] %s\n", netInfo);
  setupRoutes();
  server.begin();
}

void sendTelemetry() {
  ws.cleanupClients();
  if (ws.count() == 0) return;
  JsonDocument d;
  d["tipo"] = "telemetria";
  stateToJson(d);
  String out;
  serializeJson(d, out);
  ws.textAll(out);
}

void broadcastPlan() {
  if (ws.count() == 0) return;
  JsonDocument d;
  planToJson(d);
  String out;
  serializeJson(d, out);
  ws.textAll(out);
}

void sendEvent(const char *text) {
  if (ws.count() == 0) return;
  JsonDocument d;
  d["tipo"] = "evento";
  d["texto"] = text;
  String out;
  serializeJson(d, out);
  ws.textAll(out);
}

const char *networkInfo() { return netInfo; }

}  // namespace webapi
