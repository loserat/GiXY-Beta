// ===================== Zeltsensor v2.0 (RAW, headless) =====================
// Features:
// - Headless: kein Display/Encoder
// - SHT40 (air_temp_raw, humidity_raw)
// - BMP280 (pressure_raw, optional air_temp_bmp_raw)
// - DS18B20 (water_temp_raw)
// - AS7341 (spectrum_raw JSON + lux_raw = CLEAR-Kanal als Rohintensität)
// - OTA (ohne Passwort), MQTT (einzelne RAW-Topics), LED-Status (grün/rot)
// - Preferences speichert nur updateSec (optional), NICHT Offsets (die macht DiXY)
// ===========================================================================

#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoOTA.h>
#include <Wire.h>
#include <Preferences.h>

// SHT40
#include <Adafruit_SHT4x.h>

// BMP280
#include <Adafruit_BMP280.h>

// AS7341
#include <Adafruit_AS7341.h>

// DS18B20
#include <OneWire.h>
#include <DallasTemperature.h>

// ===================== Pin-Definitionen =====================
// I2C (SHT40, BMP280, AS7341)
#define PIN_I2C_SDA      21
#define PIN_I2C_SCL      22

// DS18B20 (1-Wire)
#define PIN_ONEWIRE      4

// LEDs (Status)
#define PIN_LED_GREEN    2    // "OK"
#define PIN_LED_RED      15   // "Fehler/OTA"

// ===================== Netzwerk & MQTT =====================
const char* WIFI_SSID   = "SSID";
const char* WIFI_PASS   = "PW";
const char* MQTT_SERVER = "192.168.177.36";
const uint16_t MQTT_PORT= 1883;

// MQTT Topics (RAW only)
const char* T_STATUS      = "dixy/tent/status";
const char* T_RSSI        = "dixy/tent/rssi";
const char* T_UPTIME      = "dixy/tent/uptime";
const char* T_CMD         = "dixy/tent/cmd";

const char* T_AIR_TEMP    = "dixy/tent/raw/air_temp";
const char* T_HUMIDITY    = "dixy/tent/raw/humidity";
const char* T_PRESSURE    = "dixy/tent/raw/pressure";
const char* T_AIR_TEMP_B  = "dixy/tent/raw/air_temp_bmp";   // optional (BMP-Temp)
const char* T_WATER_TEMP  = "dixy/tent/raw/water_temp";
const char* T_LUX_RAW     = "dixy/tent/raw/lux";            // AS7341 CLEAR-Kanal (Rohintensität)
const char* T_SPECTRUM    = "dixy/tent/raw/spectrum";       // JSON mit F1..F8,CLEAR,NIR

WiFiClient netClient;
PubSubClient mqtt(netClient);

// ===================== OTA =====================
void otaSetup();

// ===================== Preferences (nur Basissettings) =====================
Preferences prefs;
int updateSec = 30;   // Standard-Mess-/Publish-Intervall (Sekunden)

// ===================== Sensor-Objekte =====================
Adafruit_SHT4x sht4;
Adafruit_BMP280 bmp;             // I2C
Adafruit_AS7341 as7341;

OneWire oneWire(PIN_ONEWIRE);
DallasTemperature dallas(&oneWire);

// ===================== LED-Status =====================
enum LedState { LED_OFF, LED_BOOT, LED_OK, LED_ERROR, LED_OTA };
LedState ledState = LED_BOOT;
unsigned long lastLedBlink = 0;
bool ledToggle = false;

void setLedState(LedState s){ ledState = s; }
void updateLEDs(){
  unsigned long now = millis();
  int blinkMs = (ledState==LED_OTA) ? 200 : 500;
  if(now - lastLedBlink >= (unsigned long)blinkMs){
    lastLedBlink = now;
    ledToggle = !ledToggle;
  }
  switch(ledState){
    case LED_BOOT:
      digitalWrite(PIN_LED_GREEN, LOW);
      digitalWrite(PIN_LED_RED, ledToggle);
      break;
    case LED_OK:
      digitalWrite(PIN_LED_GREEN, HIGH);
      digitalWrite(PIN_LED_RED, LOW);
      break;
    case LED_ERROR:
      digitalWrite(PIN_LED_GREEN, LOW);
      digitalWrite(PIN_LED_RED, ledToggle);
      break;
    case LED_OTA:
      digitalWrite(PIN_LED_GREEN, LOW);
      digitalWrite(PIN_LED_RED, ledToggle);
      break;
    case LED_OFF:
    default:
      digitalWrite(PIN_LED_GREEN, LOW);
      digitalWrite(PIN_LED_RED, LOW);
      break;
  }
}

// ===================== Helpers =====================
void publishStr(const char* topic, const String& s, bool retain=false){
  mqtt.publish(topic, s.c_str(), retain);
}
void publishFloat(const char* topic, float v, uint8_t digits=2, bool retain=false){
  char buf[32];
  dtostrf(v, 0, digits, buf);
  mqtt.publish(topic, buf, retain);
}
void publishUInt(const char* topic, unsigned long v, bool retain=false){
  char buf[24]; snprintf(buf, sizeof(buf), "%lu", v);
  mqtt.publish(topic, buf, retain);
}

// ===================== WiFi/MQTT =====================
void wifiConnect(){
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while(WiFi.status()!=WL_CONNECTED){
    updateLEDs();
    delay(200);
  }
}
void mqttCallback(char* topic, byte* payload, unsigned int len){
  String msg; msg.reserve(len);
  for(unsigned int i=0;i<len;i++) msg += (char)payload[i];

  // Einfache Kommandos: set_update_interval:SECONDS
  if(msg.startsWith("set_update_interval:")){
    int sec = msg.substring(strlen("set_update_interval:")).toInt();
    if(sec < 5) sec = 5;
    if(sec > 900) sec = 900;
    updateSec = sec;
    prefs.begin("tent", false);
    prefs.putInt("updateSec", updateSec);
    prefs.end();
    publishStr(T_STATUS, "updateSec set to " + String(updateSec) + "s");
  }
}
void mqttConnect(){
  mqtt.setServer(MQTT_SERVER, MQTT_PORT);
  mqtt.setCallback(mqttCallback);
  while(!mqtt.connected()){
    if(mqtt.connect("ZeltsensorRAW")){
      mqtt.subscribe(T_CMD);
      publishStr(T_STATUS, "Zeltsensor RAW online (OTA ready)");
      setLedState(LED_OK);
    }else{
      setLedState(LED_ERROR);
      delay(750);
    }
  }
}

// ===================== Sensors Init =====================
bool initSHT4(){
  if(!sht4.begin()){
    publishStr(T_STATUS, "Sensor init error: SHT40");
    return false;
  }
  // Optional: Präzision einstellen (Standard OK)
  sht4.setPrecision(SHT4X_HIGH_PRECISION);
  sht4.setHeater(SHT4X_NO_HEATER);
  return true;
}
bool initBMP(){
  // Versuche 0x76, dann 0x77
  if(!bmp.begin(0x76)){
    if(!bmp.begin(0x77)){
      publishStr(T_STATUS, "Sensor init error: BMP280");
      return false;
    }
  }
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                  Adafruit_BMP280::SAMPLING_X2,   // temp
                  Adafruit_BMP280::SAMPLING_X16,  // pressure
                  Adafruit_BMP280::FILTER_X16,
                  Adafruit_BMP280::STANDBY_MS_62_5);
  return true;
}
bool initAS7341(){
  if(!as7341.begin()){
    publishStr(T_STATUS, "Sensor init error: AS7341");
    return false;
  }
  // Standard-Gain/Timing (kann später angepasst werden)
  as7341.setATIME(100);     // Integration time steps (2.78ms per step)
  as7341.setASTEP(999);     // Higher -> more light collected
  as7341.setGain(AS7341_GAIN_256X);
  return true;
}
bool initDS18(){
  dallas.begin();
  // keine feste Adresse nötig; wir nehmen Index 0
  return true;
}

// ===================== Setup =====================
void setup(){
  // Pins
  pinMode(PIN_LED_GREEN, OUTPUT);
  pinMode(PIN_LED_RED,   OUTPUT);
  setLedState(LED_BOOT);

  // I2C
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);

  // Serial
  Serial.begin(115200);

  // Preferences
  prefs.begin("tent", true);
  updateSec = prefs.getInt("updateSec", 30);
  prefs.end();

  // Sensoren
  bool okSHT  = initSHT4();
  bool okBMP  = initBMP();
  bool okAS   = initAS7341();
  bool okDS18 = initDS18();

  // Netzwerk
  wifiConnect();
  mqttConnect();
  otaSetup();

  // Status-LED je nach Sensor-Init
  if(okSHT && okBMP && okAS && okDS18) setLedState(LED_OK);
  else setLedState(LED_ERROR);
}

// ===================== OTA =====================
void otaSetup(){
  ArduinoOTA.setHostname("ZeltsensorRAW");
  // Passwort OFFEN lassen (gewünscht)
  ArduinoOTA.onStart([](){
    setLedState(LED_OTA);
    publishStr(T_STATUS, "OTA start");
  });
  ArduinoOTA.onEnd([](){
    publishStr(T_STATUS, "OTA done");
  });
  ArduinoOTA.onError([](ota_error_t error){
    publishStr(T_STATUS, "OTA error");
    setLedState(LED_ERROR);
  });
  ArduinoOTA.begin();
}

// ===================== Main Loop =====================
unsigned long lastPublish = 0;

void loop(){
  if(WiFi.status()!=WL_CONNECTED){
    setLedState(LED_ERROR);
    wifiConnect();
  }
  if(!mqtt.connected()){
    setLedState(LED_ERROR);
    mqttConnect();
  }

  mqtt.loop();
  ArduinoOTA.handle();
  updateLEDs();

  unsigned long now = millis();
  if(now - lastPublish < (unsigned long)updateSec*1000UL) return;
  lastPublish = now;

  // ---------- SHT40 ----------
  sensors_event_t humidity, temp;
  bool haveSHT = sht4.getEvent(&humidity, &temp);  // füllt beide
  if(haveSHT){
    publishFloat(T_AIR_TEMP, temp.temperature, 2);
    publishFloat(T_HUMIDITY, humidity.relative_humidity, 2);
  } else {
    publishStr(T_STATUS, "Read error: SHT40");
    setLedState(LED_ERROR);
  }

  // ---------- BMP280 ----------
  bool haveBMP = true;
  float press_hPa = NAN;
  float bmp_tC    = NAN;
  // Adafruit_BMP280 hat keine "getEvent"; direkter Abruf:
  press_hPa = bmp.readPressure() / 100.0f;  // Pa -> hPa
  bmp_tC    = bmp.readTemperature();
  if(isnan(press_hPa) || isnan(bmp_tC)) {
    publishStr(T_STATUS, "Read error: BMP280");
    haveBMP = false;
    setLedState(LED_ERROR);
  } else {
    publishFloat(T_PRESSURE,  press_hPa, 1);
    publishFloat(T_AIR_TEMP_B, bmp_tC,   2);
  }

  // ---------- DS18B20 ----------
  dallas.requestTemperatures();
  float tWater = dallas.getTempCByIndex(0);
  if(tWater > -50 && tWater < 100){
    publishFloat(T_WATER_TEMP, tWater, 2);
  } else {
    publishStr(T_STATUS, "Read error: DS18B20");
    setLedState(LED_ERROR);
  }

  // ---------- AS7341 ----------
  uint16_t readings[12]; // F1..F8, CLEAR, NIR, (Flicker not used)
  bool haveAS = as7341.readAllChannels(readings);
  if(haveAS){
    // Index-Map (laut Adafruit-Example):
    // 0:F1(415nm) 1:F2(445) 2:F3(480) 3:F4(515)
    // 4:F5(555)   5:F6(590) 6:F7(630) 7:F8(680)
    // 8:CLEAR     9:NIR     10/11 (not used here)
    uint16_t F1=readings[0], F2=readings[1], F3=readings[2], F4=readings[3];
    uint16_t F5=readings[4], F6=readings[5], F7=readings[6], F8=readings[7];
    uint16_t CLEAR=readings[8], NIR=readings[9];

    // "lux_raw": hier als CLEAR-Rohkanal veröffentlicht (nicht kalibriert!)
    publishFloat(T_LUX_RAW, (float)CLEAR, 0);

    // Spectrum JSON
    String js = "{";
    js += "\"F1\":" + String(F1);
    js += ",\"F2\":" + String(F2);
    js += ",\"F3\":" + String(F3);
    js += ",\"F4\":" + String(F4);
    js += ",\"F5\":" + String(F5);
    js += ",\"F6\":" + String(F6);
    js += ",\"F7\":" + String(F7);
    js += ",\"F8\":" + String(F8);
    js += ",\"CLEAR\":" + String(CLEAR);
    js += ",\"NIR\":" + String(NIR);
    js += "}";
    publishStr(T_SPECTRUM, js);
  } else {
    publishStr(T_STATUS, "Read error: AS7341");
    setLedState(LED_ERROR);
  }

  // ---------- Systeminfos ----------
  publishUInt(T_UPTIME, now/1000UL);
  publishFloat(T_RSSI, (float)WiFi.RSSI(), 0);

  // Wenn alles ok: LED grün an
  setLedState(LED_OK);
}
