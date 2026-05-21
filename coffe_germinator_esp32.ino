// =============================================================
//  GERMINADOR IoT — ESP32-S3
//  Telemetría → FastAPI (con HTTPS) + Setpoints + Overrides
//  + Sensor de nivel de agua (HC-SR04) con corte de bomba
// =============================================================

#include <Wire.h>
#include <Adafruit_AHTX0.h>
#include <BH1750.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <SPI.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>

#include "secrets.h"

// ===================== PANTALLA ===============================
#define TFT_CS   10
#define TFT_RST   9
#define TFT_DC    8
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);

// ===================== SENSORES ==============================
Adafruit_AHTX0 aht;
BH1750 lightMeter;

bool sensorAhtOk    = false;
bool sensorLuzOk    = false;
bool sensorNivelOk  = false;

// ===================== PINES =================================
const int pinSuelo  =  1;
const int pinSuelo2 =  3;
const int pinSuelo3 = 14;
const int pinSuelo4 = 13;
const int pinPH     =  2;

const int releVent1 =  5;
const int releVent2 =  6;
const int releLuz   =  7;
const int releBomba = 15;

const int pinTrig   = 19;
const int pinEcho   = 16;

// ===================== CALIBRACIÓN SUELO =====================
const int SUELO_SECO   = 3500;
const int SUELO_MOJADO = 1200;

// ===================== SETPOINTS =============================
float sp_temp_min            = 20.0f;
float sp_temp_vent_on        = 25.0f;
float sp_temp_vent_off       = 23.5f;
float sp_suelo_bomba_on      = 40.0f;
float sp_suelo_bomba_off     = 70.0f;
float sp_luz_on_lx           = 150.0f;
float sp_luz_off_lx          = 300.0f;
float sp_nivel_vacio_cm      = 15.0f;
float sp_nivel_lleno_cm      =  3.0f;
float sp_nivel_agua_min_pct  = 15.0f;

// ===================== OVERRIDES (modo manual) ===============
bool ovr_bomba_activo = false;  bool ovr_bomba_estado = false;
bool ovr_vent_activo  = false;  bool ovr_vent_estado  = false;
bool ovr_luz_activo   = false;  bool ovr_luz_estado   = false;

// ===================== TEMPORIZACIÓN =========================
const unsigned long INTERVALO_TELEMETRIA = 10000UL;
const unsigned long INTERVALO_PANTALLA   =  2000UL;
const unsigned long INTERVALO_RECONEXION = 15000UL;
const unsigned long INTERVALO_SETPOINTS  = 30000UL;
const unsigned long INTERVALO_OVERRIDES  = 10000UL;

// Timeout HTTP largo porque Render free tier puede tardar ~30s en despertar
// la primera vez tras inactividad. Una vez despierto, los siguientes
// requests responden en <1s.
const uint16_t TIMEOUT_HTTP_MS = 30000;

unsigned long ultimoEnvio        = 0;
unsigned long ultimaPantalla     = 0;
unsigned long ultimaReconexion   = 0;
unsigned long ultimoSetpointsGet = 0;
unsigned long ultimoOverridesGet = 0;

// ===================== ESTADO GLOBAL =========================
struct EstadoSistema {
  float temperatura        = 0.0f;
  float humedad_aire       = 0.0f;
  float humedad_suelo      = 0.0f;
  float humedad_suelo2     = 0.0f;
  float humedad_suelo3     = 0.0f;
  float humedad_suelo4     = 0.0f;
  float promedioSuelo      = 0.0f;
  float luz_lux            = 0.0f;
  float ph_simulado        = 0.0f;
  float nivel_agua_cm      = 0.0f;
  float nivel_agua_pct     = 0.0f;
  bool  estado_bomba       = false;
  bool  estado_ventilador  = false;
  bool  estado_ventilador2 = false;
  bool  bombillo           = false;
} estado;


// =============================================================
//  AUXILIARES
// =============================================================

int leerPromedio(int pin) {
  int suma = 0;
  for (int i = 0; i < 10; i++) {
    suma += analogRead(pin);
    delay(5);
  }
  return suma / 10;
}

/**
 * Decide si SERVER_BASE es HTTPS y, en ese caso, configura un
 * WiFiClientSecure con verificación de certificado deshabilitada
 * (suficiente para una demo en Render con cert válido de Let's Encrypt).
 *
 * Si SERVER_BASE empieza con "http://" usa un cliente plano.
 */
bool urlEsHttps() {
  return strncmp(SERVER_BASE, "https://", 8) == 0;
}


// =============================================================
//  WI-FI
// =============================================================

void conectarWiFi() {
  Serial.print("[WiFi] Conectando a: ");
  Serial.println(WIFI_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  uint8_t intentos = 0;
  while (WiFi.status() != WL_CONNECTED && intentos < 40) {
    delay(500);
    Serial.print(".");
    intentos++;
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n[WiFi] OK");
    Serial.print("[WiFi] IP local: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\n[WiFi] Fallo, reintentaré en el loop.");
  }
}

void asegurarWiFi() {
  if (WiFi.status() == WL_CONNECTED) return;
  unsigned long ahora = millis();
  if (ahora - ultimaReconexion < INTERVALO_RECONEXION) return;
  ultimaReconexion = ahora;
  Serial.println("[WiFi] Sin conexión, reintentando...");
  WiFi.disconnect();
  WiFi.reconnect();
}


// =============================================================
//  HC-SR04
// =============================================================

float leerDistanciaCm() {
  digitalWrite(pinTrig, LOW);
  delayMicroseconds(2);
  digitalWrite(pinTrig, HIGH);
  delayMicroseconds(10);
  digitalWrite(pinTrig, LOW);
  unsigned long duracion = pulseIn(pinEcho, HIGH, 30000UL);
  if (duracion == 0) return -1.0f;
  return (duracion * 0.0343f) / 2.0f;
}

float leerDistanciaFiltrada() {
  const int N = 5;
  float lecturas[N];
  int validas = 0;
  for (int i = 0; i < N; i++) {
    float d = leerDistanciaCm();
    if (d > 0.0f) lecturas[validas++] = d;
    delay(60);
  }
  if (validas < 3) return -1.0f;
  for (int i = 0; i < validas - 1; i++) {
    for (int j = i + 1; j < validas; j++) {
      if (lecturas[j] < lecturas[i]) {
        float t = lecturas[i]; lecturas[i] = lecturas[j]; lecturas[j] = t;
      }
    }
  }
  int ini = (validas >= 5) ? 1 : 0;
  int fin = (validas >= 5) ? validas - 1 : validas;
  float suma = 0.0f;
  for (int i = ini; i < fin; i++) suma += lecturas[i];
  return suma / (float)(fin - ini);
}

float distanciaAPorcentaje(float distancia_cm) {
  if (distancia_cm < 0.0f) return 0.0f;
  float rango = sp_nivel_vacio_cm - sp_nivel_lleno_cm;
  if (rango <= 0.0f) return 0.0f;
  float pct = (sp_nivel_vacio_cm - distancia_cm) / rango * 100.0f;
  if (pct < 0.0f)   pct = 0.0f;
  if (pct > 100.0f) pct = 100.0f;
  return pct;
}


// =============================================================
//  HTTP / HTTPS
// =============================================================

/**
 * Wrapper para hacer un HTTP POST que soporta tanto HTTP como HTTPS.
 * Devuelve el código de respuesta HTTP (o negativo si error).
 */
int httpPost(const String& url, const String& payload) {
  HTTPClient http;
  http.setTimeout(TIMEOUT_HTTP_MS);

  bool ok;
  if (urlEsHttps()) {
    WiFiClientSecure client;
    client.setInsecure();   // Demo: no validamos cert. Para producción usar cert root real.
    ok = http.begin(client, url);
  } else {
    ok = http.begin(url);
  }
  if (!ok) return -1;

  http.addHeader("Content-Type", "application/json");
  http.addHeader("X-Device-Token", DEVICE_TOKEN);

  int code = http.POST(payload);
  http.end();
  return code;
}

/**
 * Wrapper para HTTP GET con HTTPS opcional.
 * Si la respuesta es 200, llena `bodyOut`.
 */
int httpGet(const String& url, String& bodyOut) {
  HTTPClient http;
  http.setTimeout(TIMEOUT_HTTP_MS);

  bool ok;
  if (urlEsHttps()) {
    WiFiClientSecure client;
    client.setInsecure();
    ok = http.begin(client, url);
  } else {
    ok = http.begin(url);
  }
  if (!ok) return -1;

  http.addHeader("X-Device-Token", DEVICE_TOKEN);

  int code = http.GET();
  if (code == 200) bodyOut = http.getString();
  http.end();
  return code;
}


// =============================================================
//  TELEMETRÍA (POST)
// =============================================================

void enviarDatos() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[HTTP] Sin Wi-Fi, omitiendo envío.");
    return;
  }

  JsonDocument doc;
  doc["temperatura"]         = estado.temperatura;
  doc["humedad_aire"]        = estado.humedad_aire;
  doc["humedad_suelo"]       = estado.humedad_suelo;
  doc["humedad_suelo2"]      = estado.humedad_suelo2;
  doc["humedad_suelo3"]      = estado.humedad_suelo3;
  doc["humedad_suelo4"]      = estado.humedad_suelo4;
  doc["luz_lux"]             = (int)estado.luz_lux;
  doc["ph_simulado"]         = estado.ph_simulado;
  doc["estado_bomba"]        = estado.estado_bomba;
  doc["estado_ventilador"]   = estado.estado_ventilador;
  doc["estado_ventilador2"]  = estado.estado_ventilador2;
  doc["bombillo"]            = estado.bombillo;
  doc["nivel_agua_cm"]       = estado.nivel_agua_cm;
  doc["nivel_agua_pct"]      = estado.nivel_agua_pct;
  doc["sensor_aht_ok"]       = sensorAhtOk;
  doc["sensor_luz_ok"]       = sensorLuzOk;
  doc["sensor_nivel_ok"]     = sensorNivelOk;

  String payload;
  serializeJson(doc, payload);

  String url = String(SERVER_BASE) + "/api/telemetria";
  int code = httpPost(url, payload);

  if (code > 0) {
    Serial.printf("[HTTP POST] %d\n", code);
    if (code == 401 || code == 403) {
      Serial.println("[HTTP] Token inválido. Revisa secrets.h y env del backend.");
    }
  } else {
    Serial.printf("[HTTP POST] Error: %d\n", code);
  }
}


// =============================================================
//  SETPOINTS (GET)
// =============================================================

void obtenerSetpoints() {
  if (WiFi.status() != WL_CONNECTED) return;

  String url = String(SERVER_BASE) + "/api/setpoints";
  String body;
  int code = httpGet(url, body);

  if (code != 200) {
    Serial.printf("[HTTP GET setpoints] %d\n", code);
    return;
  }

  JsonDocument doc;
  if (deserializeJson(doc, body)) return;

  if (doc["temp_min"].is<float>())            sp_temp_min            = doc["temp_min"].as<float>();
  if (doc["temp_vent_on"].is<float>())        sp_temp_vent_on        = doc["temp_vent_on"].as<float>();
  if (doc["temp_vent_off"].is<float>())       sp_temp_vent_off       = doc["temp_vent_off"].as<float>();
  if (doc["suelo_bomba_on"].is<float>())      sp_suelo_bomba_on      = doc["suelo_bomba_on"].as<float>();
  if (doc["suelo_bomba_off"].is<float>())     sp_suelo_bomba_off     = doc["suelo_bomba_off"].as<float>();
  if (doc["luz_on_lx"].is<float>())           sp_luz_on_lx           = doc["luz_on_lx"].as<float>();
  if (doc["luz_off_lx"].is<float>())          sp_luz_off_lx          = doc["luz_off_lx"].as<float>();
  if (doc["nivel_vacio_cm"].is<float>())      sp_nivel_vacio_cm      = doc["nivel_vacio_cm"].as<float>();
  if (doc["nivel_lleno_cm"].is<float>())      sp_nivel_lleno_cm      = doc["nivel_lleno_cm"].as<float>();
  if (doc["nivel_agua_min_pct"].is<float>())  sp_nivel_agua_min_pct  = doc["nivel_agua_min_pct"].as<float>();
}


// =============================================================
//  OVERRIDES (GET)
// =============================================================

void obtenerOverrides() {
  if (WiFi.status() != WL_CONNECTED) return;

  String url = String(SERVER_BASE) + "/api/overrides";
  String body;
  int code = httpGet(url, body);

  if (code != 200) {
    Serial.printf("[HTTP GET overrides] %d\n", code);
    return;
  }

  JsonDocument doc;
  if (deserializeJson(doc, body)) return;
  if (!doc.is<JsonArray>()) return;

  ovr_bomba_activo = false;
  ovr_vent_activo  = false;
  ovr_luz_activo   = false;

  JsonArray arr = doc.as<JsonArray>();
  for (JsonObject o : arr) {
    const char* actuador = o["actuador"] | "";
    bool forzar          = o["forzar_estado"] | false;

    if      (strcmp(actuador, "bomba")      == 0) { ovr_bomba_activo = true; ovr_bomba_estado = forzar; }
    else if (strcmp(actuador, "ventilador") == 0) { ovr_vent_activo  = true; ovr_vent_estado  = forzar; }
    else if (strcmp(actuador, "luz")        == 0) { ovr_luz_activo   = true; ovr_luz_estado   = forzar; }
  }

  Serial.printf(
    "[Overrides] Bomba=%s Vent=%s Luz=%s\n",
    ovr_bomba_activo ? (ovr_bomba_estado ? "ON" : "OFF") : "auto",
    ovr_vent_activo  ? (ovr_vent_estado  ? "ON" : "OFF") : "auto",
    ovr_luz_activo   ? (ovr_luz_estado   ? "ON" : "OFF") : "auto"
  );
}


// =============================================================
//  PANTALLA
// =============================================================

void actualizarPantalla() {
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextSize(1);

  tft.setTextColor(ST77XX_YELLOW);
  tft.setCursor(0, 5);   tft.printf("Temp:  %.1f C", estado.temperatura);

  tft.setTextColor(ST77XX_CYAN);
  tft.setCursor(0, 18);  tft.printf("Hum:   %.1f %%", estado.humedad_aire);

  tft.setTextColor(ST77XX_GREEN);
  tft.setCursor(0, 31);  tft.printf("Luz:   %.0f lx", estado.luz_lux);

  tft.setTextColor(ST77XX_MAGENTA);
  tft.setCursor(0, 44);  tft.printf("Suelo: %.0f %%", estado.humedad_suelo);
  tft.setCursor(0, 57);  tft.printf("S2:%.0f%% S3:%.0f%%", estado.humedad_suelo2, estado.humedad_suelo3);
  tft.setCursor(0, 70);  tft.printf("S4:%.0f%%", estado.humedad_suelo4);

  tft.setTextColor(ST77XX_RED);
  tft.setCursor(0, 83);  tft.printf("pH:    %.2f", estado.ph_simulado);

  bool nivelBajo = (estado.nivel_agua_pct < sp_nivel_agua_min_pct);
  tft.setTextColor(nivelBajo ? ST77XX_RED : ST77XX_CYAN);
  tft.setCursor(0, 96);
  if (sensorNivelOk) {
    tft.printf("Agua:  %.0f%% (%.1fcm)", estado.nivel_agua_pct, estado.nivel_agua_cm);
  } else {
    tft.print("Agua:  --");
  }

  tft.setTextColor(WiFi.status() == WL_CONNECTED ? ST77XX_GREEN : ST77XX_RED);
  tft.setCursor(0, 109); tft.print(WiFi.status() == WL_CONNECTED ? "WiFi: OK" : "WiFi: --");

  if (ovr_bomba_activo || ovr_vent_activo || ovr_luz_activo) {
    tft.setTextColor(ST77XX_ORANGE);
    tft.setCursor(0, 122);
    tft.print("MANUAL ACTIVO");
  } else if (!sensorAhtOk || !sensorLuzOk || !sensorNivelOk) {
    tft.setTextColor(ST77XX_RED);
    tft.setCursor(0, 122);
    tft.print("Sensor offline");
  }
}


// =============================================================
//  SENSORES + CONTROL
// =============================================================

void leerSensoresYControlar() {
  if (sensorAhtOk) {
    sensors_event_t evtHum, evtTemp;
    aht.getEvent(&evtHum, &evtTemp);
    estado.temperatura  = evtTemp.temperature;
    estado.humedad_aire = evtHum.relative_humidity;
  }
  if (sensorLuzOk) {
    estado.luz_lux = lightMeter.readLightLevel();
  }

  auto leerSuelo = [](int pin) -> float {
    int raw = 0;
    for (int i = 0; i < 10; i++) { raw += analogRead(pin); delay(5); }
    raw /= 10;
    return (float)constrain(map(raw, SUELO_SECO, SUELO_MOJADO, 0, 100), 0, 100);
  };
  estado.humedad_suelo  = leerSuelo(pinSuelo);
  estado.humedad_suelo2 = leerSuelo(pinSuelo2);
  estado.humedad_suelo3 = leerSuelo(pinSuelo3);
  estado.humedad_suelo4 = leerSuelo(pinSuelo4);

  int phRaw          = leerPromedio(pinPH);
  estado.ph_simulado = (phRaw / 4095.0f) * 14.0f;

  float dist = leerDistanciaFiltrada();
  if (dist > 0.0f) {
    sensorNivelOk = true;
    estado.nivel_agua_cm  = dist;
    estado.nivel_agua_pct = distanciaAPorcentaje(dist);
  } else {
    sensorNivelOk = false;
    estado.nivel_agua_cm  = 0.0f;
    estado.nivel_agua_pct = 0.0f;
  }

  float promedioSuelo = (estado.humedad_suelo + estado.humedad_suelo2 +
                        estado.humedad_suelo3 + estado.humedad_suelo4) / 4.0f;
  estado.promedioSuelo = promedioSuelo;

  bool hayAguaSuficiente = estado.nivel_agua_pct >= sp_nivel_agua_min_pct;

  // VENTILADORES
  if (ovr_vent_activo) {
    bool destino = ovr_vent_estado;
    digitalWrite(releVent1, destino ? LOW : HIGH);
    digitalWrite(releVent2, destino ? LOW : HIGH);
    estado.estado_ventilador  = destino;
    estado.estado_ventilador2 = destino;
  } else {
    if (!estado.estado_ventilador && estado.temperatura > sp_temp_vent_on) {
      digitalWrite(releVent1, LOW); digitalWrite(releVent2, LOW);
      estado.estado_ventilador = true; estado.estado_ventilador2 = true;
    } else if (estado.estado_ventilador && estado.temperatura < sp_temp_vent_off) {
      digitalWrite(releVent1, HIGH); digitalWrite(releVent2, HIGH);
      estado.estado_ventilador = false; estado.estado_ventilador2 = false;
    }
  }

  // BOMBA — la seguridad de agua siempre manda
  if (ovr_bomba_activo) {
    bool destino = ovr_bomba_estado && hayAguaSuficiente;
    digitalWrite(releBomba, destino ? LOW : HIGH);
    estado.estado_bomba = destino;
  } else {
    if (!estado.estado_bomba && promedioSuelo < sp_suelo_bomba_on && hayAguaSuficiente) {
      digitalWrite(releBomba, LOW); estado.estado_bomba = true;
    } else if (estado.estado_bomba && (promedioSuelo > sp_suelo_bomba_off || !hayAguaSuficiente)) {
      digitalWrite(releBomba, HIGH); estado.estado_bomba = false;
      if (!hayAguaSuficiente) Serial.println("[Bomba] CORTE por nivel de agua bajo.");
    }
  }

  // LUZ
  if (ovr_luz_activo) {
    bool destino = ovr_luz_estado;
    digitalWrite(releLuz, destino ? LOW : HIGH);
    estado.bombillo = destino;
  } else {
    if (estado.temperatura < sp_temp_min) {
      digitalWrite(releLuz, LOW); estado.bombillo = true;
    } else if (!estado.bombillo && estado.luz_lux < sp_luz_on_lx) {
      digitalWrite(releLuz, LOW); estado.bombillo = true;
    } else if (estado.bombillo && estado.luz_lux > sp_luz_off_lx) {
      digitalWrite(releLuz, HIGH); estado.bombillo = false;
    }
  }

  Serial.printf(
    "T:%.1f H:%.1f L:%.0f S(avg):%.0f Agua:%.0f%% Bomba:%d%s Vent:%d%s Luz:%d%s\n",
    estado.temperatura, estado.humedad_aire, estado.luz_lux,
    promedioSuelo, estado.nivel_agua_pct,
    estado.estado_bomba,      ovr_bomba_activo ? "*" : "",
    estado.estado_ventilador, ovr_vent_activo  ? "*" : "",
    estado.bombillo,          ovr_luz_activo   ? "*" : ""
  );
}


// =============================================================
//  SETUP / LOOP
// =============================================================

void setup() {
  Serial.begin(115200);

  Wire.begin(17, 18);
  sensorAhtOk = aht.begin();
  if (!sensorAhtOk) Serial.println("[Sensor] AHT no responde.");
  sensorLuzOk = lightMeter.begin();
  if (!sensorLuzOk) Serial.println("[Sensor] BH1750 no responde.");

  SPI.begin(12, -1, 11, TFT_CS);
  tft.initR(INITR_BLACKTAB);
  tft.setRotation(1);
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextColor(ST77XX_WHITE);
  tft.setCursor(0, 0);
  tft.print("Iniciando...");

  pinMode(releVent1, OUTPUT); digitalWrite(releVent1, HIGH);
  pinMode(releVent2, OUTPUT); digitalWrite(releVent2, HIGH);
  pinMode(releLuz,   OUTPUT); digitalWrite(releLuz,   HIGH);
  pinMode(releBomba, OUTPUT); digitalWrite(releBomba, HIGH);

  pinMode(pinTrig, OUTPUT);
  pinMode(pinEcho, INPUT);
  digitalWrite(pinTrig, LOW);

  conectarWiFi();

  Serial.printf("[Sistema] Modo conexión: %s\n", urlEsHttps() ? "HTTPS" : "HTTP");

  obtenerSetpoints();
  obtenerOverrides();

  Serial.println("[Sistema] Listo.");
}

void loop() {
  unsigned long ahora = millis();
  asegurarWiFi();

  if (ahora - ultimaPantalla >= INTERVALO_PANTALLA) {
    ultimaPantalla = ahora;
    leerSensoresYControlar();
    actualizarPantalla();
  }
  if (ahora - ultimoEnvio >= INTERVALO_TELEMETRIA) {
    ultimoEnvio = ahora;
    enviarDatos();
  }
  if (ahora - ultimoSetpointsGet >= INTERVALO_SETPOINTS) {
    ultimoSetpointsGet = ahora;
    obtenerSetpoints();
  }
  if (ahora - ultimoOverridesGet >= INTERVALO_OVERRIDES) {
    ultimoOverridesGet = ahora;
    obtenerOverrides();
  }
}
