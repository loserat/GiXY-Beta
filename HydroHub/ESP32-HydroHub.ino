// ===================== HydroHub v2.0 (ESP32) =====================
// Features: EC + pH (Kalibrierung 3-2-3), 2x DS18B20 mit Offset,
// OLED (U8G2), Drehencoder-Menü, MQTT, OTA, Kalibrier-Erinnerung.
// ================================================================

#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoOTA.h>
#include <Wire.h>
#include <U8g2lib.h>
#include <Preferences.h>
#include <OneWire.h>
#include <DallasTemperature.h>

// ===================== Pin-Definitionen =====================
// I2C (OLED)
#define PIN_OLED_SDA   21
#define PIN_OLED_SCL   22

// Analog Sensoren
#define PIN_EC_ANALOG  34
#define PIN_PH_ANALOG  35

// Temperatur-Sensoren (getrennte OneWire-Busse)
#define PIN_TEMP1      4    // Tank
#define PIN_TEMP2      5    // Zelt/Rücklauf

// Encoder / Buttons
#define PIN_ENC_A      25
#define PIN_ENC_B      26
#define PIN_ENC_BTN    32
#define PIN_ENC_BACK   33

// Optional freie Pins (Relais etc.)
// #define PIN_RELAY1   12
// #define PIN_RELAY2   13

// ===================== Netzwerk & MQTT =====================
const char* WIFI_SSID     = "SSID";
const char* WIFI_PASS     = "PW";
const char* MQTT_SERVER   = "192.168.177.36";
const uint16_t MQTT_PORT  = 1883;
// (Optional) MQTT-Auth:
// const char* MQTT_USER  = "";
// const char* MQTT_PASSW = "";

WiFiClient espClient;
PubSubClient mqtt(espClient);

// Topics
const char* T_EC        = "dixy/hydro/ec";       // mS/cm @25C
const char* T_PH        = "dixy/hydro/ph";
const char* T_T1        = "dixy/hydro/temp1";    // °C korrigiert
const char* T_T2        = "dixy/hydro/temp2";
const char* T_STATUS    = "dixy/hydro/status";   // Textstatus
const char* T_CMD       = "dixy/hydro/cmd";      // Kommandos

// ===================== Display =====================
U8G2_SSD1309_128X64_NONAME0_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);

// ===================== OneWire / Temps =====================
OneWire ow1(PIN_TEMP1);
OneWire ow2(PIN_TEMP2);
DallasTemperature dt1(&ow1);
DallasTemperature dt2(&ow2);

// Temperatur-Offsets (±2.0°C)
float t1_offset = 0.0f;
float t2_offset = 0.0f;

// ===================== EC/pH Kalibrierung & Messung =====================
// EC-Default Rohspannungen (bei 1413 µS/cm & 12.88 mS/cm)
float ec_rawLow_default  = 0.104f;
float ec_rawHigh_default = 1.598f;
float ec_rawLow  = 0.104f;
float ec_rawHigh = 1.598f;

// pH-Default Rohspannungen (PH-4502C grob; echte Werte per Kalibrierung)
float ph7_default = 2.50f;
float ph4_default = 3.00f;
float ph7_v = 2.50f;
float ph4_v = 3.00f;

// Glättung EC/pH
const int NUM_SAMPLES = 20;
float ecSamples[NUM_SAMPLES]; int ecIdx=0;
float phSamples[NUM_SAMPLES]; int phIdx=0;

// Mess-Intervalle
int updateSec  = 20;  // Homescreen/MQTT
int homeSec    = 60;  // Menü → Homescreen
int displaySec = 60;  // Display-Sleep

// Kalibrier-Erinnerung
int   calibReminderDays = 14;   // einstellbar (1..60)
unsigned long lastCalibEC_ms = 0;
unsigned long lastCalibPH_ms = 0;
unsigned long lastReminderCheck_ms = 0;

// Timings (ms)
unsigned long updateInterval;
unsigned long menuTimeout;
unsigned long displayTimeout;

// EC @25°C Kompensation
float ecTo25C(float ec_meas, float tempC) {
  if (isnan(tempC)) return ec_meas;
  const float alpha = 0.02f;
  float factor = 1.0f + alpha * (tempC - 25.0f);
  if (factor <= 0.1f) factor = 0.1f;
  return ec_meas / factor;
}

static inline float adcVoltage(int pin) {
  int raw = analogRead(pin);
  return (raw / 4095.0f) * 3.3f;
}

float ecFromVoltage(float v) {
  float slope  = (12.88f - 1.413f) / (ec_rawHigh - ec_rawLow);
  float offset = 1.413f - slope * ec_rawLow;
  float ec = slope * v + offset;
  if (ec < 0) ec = 0;
  return ec;
}
float phFromVoltage(float u) {
  // Gerade durch (ph7_v,7.00) und (ph4_v,4.00)
  float m = (4.00f - 7.00f) / (ph4_v - ph7_v);
  return m * (u - ph7_v) + 7.00f;
}

float smoothPush(float* buf, int& idx, int N, float v) {
  buf[idx] = v; idx = (idx + 1) % N;
  float s=0; for (int i=0;i<N;i++) s+=buf[i];
  return s / N;
}

// ===================== Preferences =====================
Preferences prefs;

// ===================== Encoder / Menü =====================
enum MenuState {
  HOMESCREEN, MAINMENU, CALIB_MENU, SETTINGS_MENU,
  INFO_SCREEN, HISTORY_SCREEN, EDITING, CALIBRATING
};
MenuState menuState = HOMESCREEN;

int menuIndex = 0;
bool editing = false;
int  editingWhich = -1;
int  tempEditInt = 0;    // für Sekunden/Tage
float tempEditFloat = 0; // für Offsets

// Menüs
const char* mainMenu[] = {"Kalibrierung","Einstellungen","Info","Verlauf","Zurueck"};
const int mainLen=5;

const char* calibMenu[]={
  "EC Kalibrieren",
  "pH Kalibrieren",
  "Temp1 Offset",
  "Temp2 Offset",
  "Reset EC",
  "Reset pH",
  "Zurueck"
};
const int calibLen=7;

const char* settingsMenu[]={
  "Update",
  "Homescreen",
  "Display",
  "Erinnerung (Tage)",
  "Zurueck"
};
const int settingsLen=5;

// Encoder
int lastA = HIGH;
unsigned long lastStep=0;
const unsigned long stepDelay=200;
unsigned long btnPressTime=0;
bool btnPressed=false;

// Display-Power Mgmt
bool displayOn=true;
unsigned long lastDisplay=0;
unsigned long lastInteraction=0;
unsigned long lastUpdate=0;
unsigned long lastHistory=0;

// Verlauf (EC)
const int HISTORY_SIZE=60; // 30min bei 30s
float ecHistory[HISTORY_SIZE];
int   histIdx=0;

// ===================== Kalibrier-State-Machine =====================
enum CalibType { CALIB_EC, CALIB_PH, CALIB_NONE };
struct CalibSM {
  bool active=false;
  CalibType type=CALIB_NONE;
  int phase=0; // 0 idle, 1 A(3min), 2 pause(2min), 3 B(3min), 4 done
  unsigned long phaseStart=0;
  // Sammeln der letzten 30s der jeweiligen 3min-Phasen
  float sumLast=0.0f;
  int   cntLast=0;
  // Ergebnisse (Spannungen)
  float vA=0.0f; // EC: low / pH: 7.00
  float vB=0.0f; // EC: high / pH: 4.00
} calib;

const unsigned long CAL_PHASE_MS = 3UL*60UL*1000UL;
const unsigned long CAL_PAUSE_MS = 2UL*60UL*1000UL;
const unsigned long CAL_AVG_WINDOW_MS = 30UL*1000UL;

// ===================== Hilfs-Draws =====================
void wakeDisplay(){
  if(!displayOn){ u8g2.setPowerSave(0); displayOn=true; }
  lastDisplay = millis();
}

void drawMenuList(const char* items[], int len){
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x12_tr);
  for(int i=0;i<len;i++){
    if(i==menuIndex) u8g2.drawStr(2,(i+1)*12,">");
    u8g2.drawStr(12,(i+1)*12,items[i]);
  }
  u8g2.sendBuffer();
}

void drawHome(float ec25, float ph, float t1, float t2){
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x12_tr);
  u8g2.drawStr(0,10,"HydroHub v2.0");
  u8g2.drawHLine(0,12,128);

  char buf[40];
  snprintf(buf,sizeof(buf),"EC : %.2f mS/cm   pH : %.2f", ec25, ph);
  u8g2.drawStr(0,28,buf);
  snprintf(buf,sizeof(buf),"T1 : %.1fC        T2 : %.1fC", t1, t2);
  u8g2.drawStr(0,44,buf);
  u8g2.drawStr(0,60,"@25C kompensiert");
  u8g2.sendBuffer();
}

void drawInfo(float ec25, float ph, float t1, float t2){
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x12_tr);
  u8g2.drawStr(2,12,"Firmware: v2.0");
  char buf[44];
  snprintf(buf,sizeof(buf),"IP: %s", WiFi.localIP().toString().c_str()); u8g2.drawStr(2,24,buf);
  snprintf(buf,sizeof(buf),"EC25: %.2f mS/cm", ec25); u8g2.drawStr(2,36,buf);
  snprintf(buf,sizeof(buf),"pH: %.2f  T1:%.1fC T2:%.1fC", ph, t1, t2); u8g2.drawStr(2,48,buf);
  u8g2.drawStr(2,60,"Back = zurueck");
  u8g2.sendBuffer();
}

void drawHistory(){
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x12_tr);
  u8g2.drawStr(2,10,"EC Verlauf 30min");

  float mn=9999,mx=-9999;
  for(int i=0;i<HISTORY_SIZE;i++){ if(ecHistory[i]<mn) mn=ecHistory[i]; if(ecHistory[i]>mx) mx=ecHistory[i]; }
  if(mn==9999){ mn=0; mx=1; }
  if(mn==mx){ mn-=0.1f; mx+=0.1f; }

  for(int i=0;i<HISTORY_SIZE;i++){
    int x = map(i,0,HISTORY_SIZE-1,0,127);
    long vy = lroundf(ecHistory[i]*100.0f);
    long vmin=lroundf(mn*100.0f), vmax=lroundf(mx*100.0f);
    int y = map(vy,vmin,vmax,60,20);
    if(y<0) y=0; if(y>63) y=63;
    u8g2.drawPixel(x,y);
  }
  u8g2.sendBuffer();
}

void drawEditInt(const char* title, int v, const char* unit){
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x12_tr);
  char buf[40];
  snprintf(buf,sizeof(buf),"%s: %d%s",title,v,unit?unit:"");
  u8g2.drawStr(2,24,buf);
  u8g2.drawStr(2,48,"< drehen  |  Klick=OK");
  u8g2.sendBuffer();
}
void drawEditFloat(const char* title, float v, const char* unit){
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x12_tr);
  char buf[40];
  snprintf(buf,sizeof(buf),"%s: %+0.1f%s",title,v,unit?unit:"");
  u8g2.drawStr(2,24,buf);
  u8g2.drawStr(2,48,"< drehen  |  Klick=OK");
  u8g2.sendBuffer();
}

void drawCalibScreen(const char* header, const char* line2, unsigned long remain_ms){
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x12_tr);
  u8g2.drawStr(2,14,header);
  u8g2.drawStr(2,28,line2);
  unsigned long s = (remain_ms/1000UL);
  char buf[32]; snprintf(buf,sizeof(buf),"Verbleibend: %02lu:%02lu", s/60, s%60);
  u8g2.drawStr(2,44,buf);
  u8g2.drawStr(2,60,"Klick: Abbrechen");
  u8g2.sendBuffer();
}

void drawCalibDone(const char* title, float vA, float vB, const char* labelA, const char* labelB){
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x12_tr);
  u8g2.drawStr(2,12,title);
  char buf[40];
  snprintf(buf,sizeof(buf),"%s = %.3f V", labelA, vA); u8g2.drawStr(2,28,buf);
  snprintf(buf,sizeof(buf),"%s = %.3f V", labelB, vB); u8g2.drawStr(2,44,buf);
  u8g2.drawStr(2,60,"Fertig");
  u8g2.sendBuffer();
}

// ===================== WiFi/MQTT/OTA =====================
void wifiConnect(){
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while(WiFi.status()!=WL_CONNECTED){ delay(300); }
}
void mqttConnect(){
  mqtt.setServer(MQTT_SERVER, MQTT_PORT);
  mqtt.setCallback([](char* topic, byte* payload, unsigned int length){
    String cmd; cmd.reserve(length);
    for(unsigned int i=0;i<length;i++) cmd+=(char)payload[i];

    if(cmd=="calibrate_ec"){
      if(!calib.active){ calib={true,CALIB_EC,1,millis(),0,0,0,0}; menuState=CALIBRATING; }
    } else if(cmd=="calibrate_ph"){
      if(!calib.active){ calib={true,CALIB_PH,1,millis(),0,0,0,0}; menuState=CALIBRATING; }
    } else if(cmd=="reset_ec"){
      ec_rawLow=ec_rawLow_default; ec_rawHigh=ec_rawHigh_default;
      prefs.begin("ec",false); prefs.putFloat("rawLow",ec_rawLow); prefs.putFloat("rawHigh",ec_rawHigh); prefs.end();
      mqtt.publish(T_STATUS,"EC defaults restored");
    } else if(cmd=="reset_ph"){
      ph7_v=ph7_default; ph4_v=ph4_default;
      prefs.begin("ph",false); prefs.putFloat("ph7_v",ph7_v); prefs.putFloat("ph4_v",ph4_v); prefs.end();
      mqtt.publish(T_STATUS,"pH defaults restored");
    } else if(cmd.startsWith("set_temp1_offset:")){
      float v=cmd.substring(17).toFloat();
      if(v<-2.0f) v=-2.0f; if(v>2.0f) v=2.0f;
      t1_offset=v; prefs.begin("temp",false); prefs.putFloat("t1_offset",t1_offset); prefs.end();
    } else if(cmd.startsWith("set_temp2_offset:")){
      float v=cmd.substring(17).toFloat();
      if(v<-2.0f) v=-2.0f; if(v>2.0f) v=2.0f;
      t2_offset=v; prefs.begin("temp",false); prefs.putFloat("t2_offset",t2_offset); prefs.end();
    }
  });

  while(!mqtt.connected()){
    if(mqtt.connect("HydroHubESP32"/*,MQTT_USER,MQTT_PASSW*/)){
      mqtt.subscribe(T_CMD);
      mqtt.publish(T_STATUS,"HydroHub v2.0 online (OTA ready)");
    } else { delay(1000); }
  }
}
void otaSetup(){
  ArduinoOTA.setHostname("HydroHub");
  // Passwort offen lassen (gewünscht)
  ArduinoOTA.onStart([](){
    u8g2.clearBuffer(); u8g2.setFont(u8g2_font_6x12_tr);
    u8g2.drawStr(2,34,"OTA Update..."); u8g2.sendBuffer();
    mqtt.publish(T_STATUS,"OTA update started");
  });
  ArduinoOTA.onEnd([](){
    mqtt.publish(T_STATUS,"OTA update done");
  });
  ArduinoOTA.onError([](ota_error_t error){
    mqtt.publish(T_STATUS,"OTA error");
  });
  ArduinoOTA.begin();
}

// ===================== Setup =====================
void applyMillis(){
  updateInterval = (unsigned long)updateSec*1000UL;
  menuTimeout    = (unsigned long)homeSec*1000UL;
  displayTimeout = (unsigned long)displaySec*1000UL;
}
void setup(){
  // ADC Range
  analogReadResolution(12);
  analogSetPinAttenuation(PIN_EC_ANALOG, ADC_11db);
  analogSetPinAttenuation(PIN_PH_ANALOG, ADC_11db);

  // IO
  pinMode(PIN_ENC_A, INPUT_PULLUP);
  pinMode(PIN_ENC_B, INPUT_PULLUP);
  pinMode(PIN_ENC_BTN, INPUT_PULLUP);
  pinMode(PIN_ENC_BACK, INPUT_PULLUP);

  // Serial
  Serial.begin(115200);

  // Display
  Wire.begin(PIN_OLED_SDA, PIN_OLED_SCL);
  u8g2.begin(); u8g2.setPowerSave(0);
  u8g2.setFont(u8g2_font_6x12_tr);
  u8g2.clearBuffer(); u8g2.drawStr(2,34,"Starte..."); u8g2.sendBuffer();

  // OneWire Temps
  dt1.begin(); dt2.begin();

  // Load prefs
  prefs.begin("ec", true);
  ec_rawLow  = prefs.getFloat("rawLow",  ec_rawLow_default);
  ec_rawHigh = prefs.getFloat("rawHigh", ec_rawHigh_default);
  prefs.end();

  prefs.begin("ph", true);
  ph7_v = prefs.getFloat("ph7_v", ph7_default);
  ph4_v = prefs.getFloat("ph4_v", ph4_default);
  prefs.end();

  prefs.begin("temp", true);
  t1_offset = prefs.getFloat("t1_offset", 0.0f);
  t2_offset = prefs.getFloat("t2_offset", 0.0f);
  prefs.end();

  prefs.begin("settings", true);
  calibReminderDays = prefs.getInt("calib_reminder", 14);
  updateSec  = prefs.getInt("updateSec", 20);
  homeSec    = prefs.getInt("homeSec", 60);
  displaySec = prefs.getInt("displaySec", 60);
  prefs.end();

  // Init buffers
  for(int i=0;i<NUM_SAMPLES;i++){ ecSamples[i]=0; phSamples[i]=0; }
  for(int i=0;i<HISTORY_SIZE;i++) ecHistory[i]=0;

  applyMillis();
  lastInteraction = millis();
  lastDisplay = millis();

  // Network
  wifiConnect();
  mqttConnect();
  otaSetup();

  // Ready
  u8g2.clearBuffer(); u8g2.drawStr(2,34,"Bereit"); u8g2.sendBuffer();
  delay(400);
}

// ===================== Helpers =====================
void publishFloat(const char* topic, float v, int digits=2){
  char p[24]; dtostrf(v, 0, digits, p); mqtt.publish(topic, p, true);
}

void startCalibration(CalibType t){
  calib.active=true; calib.type=t; calib.phase=1; calib.phaseStart=millis();
  calib.sumLast=0; calib.cntLast=0; calib.vA=0; calib.vB=0;
  menuState = CALIBRATING;
  mqtt.publish(T_STATUS, t==CALIB_EC ? "EC calibration step 1/2 starting" : "pH calibration step 1/2 starting");
}

void abortCalibration(){
  calib.active=false; calib.type=CALIB_NONE; calib.phase=0;
  mqtt.publish(T_STATUS,"Calibration aborted");
  menuState = MAINMENU; menuIndex=0; drawMenuList(mainMenu,mainLen);
}

// Während Kalibrierung kontinuierlich messen und in den letzten 30s mitteln
void calibTick(){
  if(!calib.active) return;
  unsigned long now=millis();
  unsigned long elapsed = now - calib.phaseStart;

  // Ermittlung Restzeit je Phase
  unsigned long phaseDur = (calib.phase==1 || calib.phase==3) ? CAL_PHASE_MS :
                           (calib.phase==2 ? CAL_PAUSE_MS : 0);
  unsigned long remain = (elapsed < phaseDur) ? (phaseDur - elapsed) : 0;

  // Anzeige
  if(calib.type==CALIB_EC){
    if(calib.phase==1)      drawCalibScreen("EC Kalibrierung (1/2)","1413 uS/cm Loesung", remain);
    else if(calib.phase==2) drawCalibScreen("EC Kalibrierung","Spuelen / Pause", remain);
    else if(calib.phase==3) drawCalibScreen("EC Kalibrierung (2/2)","12.88 mS/cm Loesung", remain);
  } else {
    if(calib.phase==1)      drawCalibScreen("pH Kalibrierung (1/2)","pH 7.00 Loesung", remain);
    else if(calib.phase==2) drawCalibScreen("pH Kalibrierung","Spuelen / Pause", remain);
    else if(calib.phase==3) drawCalibScreen("pH Kalibrierung (2/2)","pH 4.00 Loesung", remain);
  }

  // Messfenster: nur in Phasen 1 und 3; mitteln über letzte 30s innerhalb der 3min
  if(calib.phase==1 || calib.phase==3){
    // in den letzten 30s sammeln
    if(remain <= CAL_AVG_WINDOW_MS){
      float v = adcVoltage(calib.type==CALIB_EC ? PIN_EC_ANALOG : PIN_PH_ANALOG);
      calib.sumLast += v;
      calib.cntLast += 1;
    }
  }

  // Phasenwechsel
  if(elapsed >= phaseDur){
    if(calib.phase==1){
      // Ende Phase 1 → vA speichern
      float avg = (calib.cntLast>0) ? (calib.sumLast / calib.cntLast) : adcVoltage(calib.type==CALIB_EC?PIN_EC_ANALOG:PIN_PH_ANALOG);
      calib.vA = avg;
      calib.sumLast=0; calib.cntLast=0;
      calib.phase=2; calib.phaseStart=now;
      mqtt.publish(T_STATUS, calib.type==CALIB_EC ? "EC calibration pause 2min" : "pH calibration pause 2min");
    } else if(calib.phase==2){
      calib.phase=3; calib.phaseStart=now;
      mqtt.publish(T_STATUS, calib.type==CALIB_EC ? "EC calibration step 2/2 starting" : "pH calibration step 2/2 starting");
    } else if(calib.phase==3){
      float avg = (calib.cntLast>0) ? (calib.sumLast / calib.cntLast) : adcVoltage(calib.type==CALIB_EC?PIN_EC_ANALOG:PIN_PH_ANALOG);
      calib.vB = avg;
      calib.phase=4; // done

      // Persistieren & Anzeige
      if(calib.type==CALIB_EC){
        ec_rawLow  = calib.vA;
        ec_rawHigh = calib.vB;
        prefs.begin("ec",false);
        prefs.putFloat("rawLow", ec_rawLow);
        prefs.putFloat("rawHigh", ec_rawHigh);
        prefs.end();
        lastCalibEC_ms = millis();
        drawCalibDone("EC Kalibrierung OK", ec_rawLow, ec_rawHigh, "1413", "12.88");
        mqtt.publish(T_STATUS,"EC calibration done");
      } else {
        ph7_v = calib.vA;
        ph4_v = calib.vB;
        prefs.begin("ph",false);
        prefs.putFloat("ph7_v", ph7_v);
        prefs.putFloat("ph4_v", ph4_v);
        prefs.end();
        lastCalibPH_ms = millis();
        drawCalibDone("pH Kalibrierung OK", ph7_v, ph4_v, "pH7", "pH4");
        mqtt.publish(T_STATUS,"pH calibration done");
      }

      // nach kurzer Pause zurück
      delay(1500);
      calib.active=false; calib.type=CALIB_NONE; calib.phase=0;
      menuState = MAINMENU; menuIndex=0;
      drawMenuList(mainMenu,mainLen);
    }
  }
}

// ===================== Input Handling =====================
void handleRotation(int dir){
  if(menuState==MAINMENU){
    menuIndex = (menuIndex + (dir>0?1:-1) + mainLen) % mainLen;
    drawMenuList(mainMenu,mainLen);
  } else if(menuState==CALIB_MENU && !editing){
    menuIndex = (menuIndex + (dir>0?1:-1) + calibLen) % calibLen;
    drawMenuList(calibMenu,calibLen);
  } else if(menuState==SETTINGS_MENU && !editing){
    menuIndex = (menuIndex + (dir>0?1:-1) + settingsLen) % settingsLen;
    drawMenuList(settingsMenu,settingsLen);
  } else if(menuState==SETTINGS_MENU && editing){
    // Edit ints (Sekunden/Tage)
    int delta = (dir>0?1:-1);
    if(editingWhich==0 || editingWhich==1 || editingWhich==2){
      tempEditInt += (delta*5); // in 5s-Schritten
      if(tempEditInt<5) tempEditInt=5;
      if(tempEditInt>300) tempEditInt=300;
      const char* titles[]={"Update","Homescreen","Display","Erinnerung"};
      drawEditInt(titles[editingWhich], tempEditInt, "s");
    } else if(editingWhich==3){
      tempEditInt += delta; // Tage
      if(tempEditInt<1) tempEditInt=1;
      if(tempEditInt>60) tempEditInt=60;
      drawEditInt("Erinnerung", tempEditInt, "d");
    }
  } else if(menuState==CALIB_MENU && editing){
    // Temp Offsets
    int delta = (dir>0?1:-1);
    tempEditFloat += delta*0.1f;
    if(tempEditFloat<-2.0f) tempEditFloat=-2.0f;
    if(tempEditFloat> 2.0f) tempEditFloat= 2.0f;
    if(editingWhich==10) drawEditFloat("Temp1 Offset", tempEditFloat, "C");
    if(editingWhich==11) drawEditFloat("Temp2 Offset", tempEditFloat, "C");
  }
}

void startEditingInt(int which, int curr){
  editing=true; editingWhich=which; tempEditInt=curr;
  const char* titles[]={"Update","Homescreen","Display","Erinnerung"};
  if(which==3) drawEditInt("Erinnerung", tempEditInt, "d");
  else drawEditInt(titles[which], tempEditInt, "s");
}
void startEditingOffset(int which, float curr){
  editing=true; editingWhich=which; tempEditFloat=curr;
  if(which==10) drawEditFloat("Temp1 Offset", tempEditFloat, "C");
  if(which==11) drawEditFloat("Temp2 Offset", tempEditFloat, "C");
}
void saveEditing(){
  if(!editing) return;
  if(menuState==SETTINGS_MENU){
    if(editingWhich==0){ updateSec=tempEditInt; prefs.begin("settings",false); prefs.putInt("updateSec",updateSec); prefs.end(); }
    else if(editingWhich==1){ homeSec=tempEditInt; prefs.begin("settings",false); prefs.putInt("homeSec",homeSec); prefs.end(); }
    else if(editingWhich==2){ displaySec=tempEditInt; prefs.begin("settings",false); prefs.putInt("displaySec",displaySec); prefs.end(); }
    else if(editingWhich==3){ calibReminderDays=tempEditInt; prefs.begin("settings",false); prefs.putInt("calib_reminder",calibReminderDays); prefs.end(); }
    applyMillis();
    drawMenuList(settingsMenu,settingsLen);
  } else if(menuState==CALIB_MENU){
    if(editingWhich==10){ t1_offset=tempEditFloat; prefs.begin("temp",false); prefs.putFloat("t1_offset",t1_offset); prefs.end(); }
    if(editingWhich==11){ t2_offset=tempEditFloat; prefs.begin("temp",false); prefs.putFloat("t2_offset",t2_offset); prefs.end(); }
    drawMenuList(calibMenu,calibLen);
  }
  editing=false; editingWhich=-1;
}
void cancelEditing(){
  editing=false; editingWhich=-1;
  if(menuState==SETTINGS_MENU) drawMenuList(settingsMenu,settingsLen);
  if(menuState==CALIB_MENU)    drawMenuList(calibMenu,calibLen);
}

// ===================== Loop =====================
void loop(){
  if(!mqtt.connected()) mqttConnect();
  mqtt.loop();
  ArduinoOTA.handle();

  unsigned long now = millis();

  // Encoder A Flanke
  int a = digitalRead(PIN_ENC_A);
  int b = digitalRead(PIN_ENC_B);
  if(a!=lastA && a==LOW){
    if(now - lastStep > stepDelay){
      lastStep = now; lastInteraction=now; wakeDisplay();
      int dir = (b==HIGH)? +1 : -1;
      if(menuState!=CALIBRATING) handleRotation(dir);
    }
  }
  lastA=a;

  // Encoder Button
  if(digitalRead(PIN_ENC_BTN)==LOW){
    if(!btnPressed){ btnPressed=true; btnPressTime=now; }
  } else {
    if(btnPressed){
      unsigned long press = now - btnPressTime; btnPressed=false; lastInteraction=now; wakeDisplay();
      if(press<2000){
        // Kurz-Klick
        if(menuState==HOMESCREEN){ menuState=MAINMENU; menuIndex=0; drawMenuList(mainMenu,mainLen); }
        else if(menuState==MAINMENU){
          if(menuIndex==0){ menuState=CALIB_MENU; menuIndex=0; drawMenuList(calibMenu,calibLen); }
          else if(menuIndex==1){ menuState=SETTINGS_MENU; menuIndex=0; drawMenuList(settingsMenu,settingsLen); }
          else if(menuIndex==2){ menuState=INFO_SCREEN; /* wird beim Tick gezeichnet */ }
          else if(menuIndex==3){ menuState=HISTORY_SCREEN; drawHistory(); }
          else { menuState=HOMESCREEN; }
        }
        else if(menuState==CALIB_MENU){
          if(!editing){
            if(menuIndex==0){ startCalibration(CALIB_EC); }
            else if(menuIndex==1){ startCalibration(CALIB_PH); }
            else if(menuIndex==2){ startEditingOffset(10, t1_offset); }
            else if(menuIndex==3){ startEditingOffset(11, t2_offset); }
            else if(menuIndex==4){
              ec_rawLow=ec_rawLow_default; ec_rawHigh=ec_rawHigh_default;
              prefs.begin("ec",false); prefs.putFloat("rawLow",ec_rawLow); prefs.putFloat("rawHigh",ec_rawHigh); prefs.end();
              drawMenuList(calibMenu,calibLen);
            }
            else if(menuIndex==5){
              ph7_v=ph7_default; ph4_v=ph4_default;
              prefs.begin("ph",false); prefs.putFloat("ph7_v",ph7_v); prefs.putFloat("ph4_v",ph4_v); prefs.end();
              drawMenuList(calibMenu,calibLen);
            }
            else { menuState=MAINMENU; menuIndex=0; drawMenuList(mainMenu,mainLen); }
          } else {
            saveEditing();
          }
        }
        else if(menuState==SETTINGS_MENU){
          if(!editing){
            if(menuIndex==0) startEditingInt(0, updateSec);
            else if(menuIndex==1) startEditingInt(1, homeSec);
            else if(menuIndex==2) startEditingInt(2, displaySec);
            else if(menuIndex==3) startEditingInt(3, calibReminderDays);
            else { menuState=MAINMENU; menuIndex=0; drawMenuList(mainMenu,mainLen); }
          } else {
            saveEditing();
          }
        }
        else if(menuState==INFO_SCREEN){
          menuState=MAINMENU; menuIndex=0; drawMenuList(mainMenu,mainLen);
        }
        else if(menuState==HISTORY_SCREEN){
          menuState=MAINMENU; menuIndex=0; drawMenuList(mainMenu,mainLen);
        }
        else if(menuState==CALIBRATING){
          // Abbruch mit Rückfrage -> hier sofort abbrechen (einfach)
          abortCalibration();
        }
      }
    }
  }

  // Back
  if(digitalRead(PIN_ENC_BACK)==LOW){
    wakeDisplay();
    if(menuState==SETTINGS_MENU && editing)      cancelEditing();
    else if(menuState==CALIB_MENU && editing)    cancelEditing();
    else if(menuState==CALIBRATING)              abortCalibration();
    else if(menuState==MAINMENU){ menuState=HOMESCREEN; }
    else if(menuState==CALIB_MENU || menuState==SETTINGS_MENU || menuState==INFO_SCREEN || menuState==HISTORY_SCREEN){
      menuState=MAINMENU; menuIndex=0; drawMenuList(mainMenu,mainLen);
    }
    lastInteraction=now; delay(160);
  }

  // Display Timeout
  if(displayOn && (now - lastDisplay > displayTimeout)){ u8g2.setPowerSave(1); displayOn=false; }

  // Kalibrier-State läuft unabhängig
  if(menuState==CALIBRATING) calibTick();

  // Regelmäßige Mess-/Publish-Updates
  static float last_ec25=0, last_ph=7, last_t1=NAN, last_t2=NAN;
  static unsigned long lastTempReq=0;

  // DS18B20: alle ~1s abfragen (schnell genug)
  if(now - lastTempReq > 1000){
    dt1.requestTemperatures();
    dt2.requestTemperatures();
    float t1 = dt1.getTempCByIndex(0);
    float t2 = dt2.getTempCByIndex(0);
    if(t1>-100 && t1<100) last_t1 = t1 + t1_offset;
    if(t2>-100 && t2<100) last_t2 = t2 + t2_offset;
    lastTempReq = now;
  }

  // Homescreen/MQTT Update
  if(menuState==HOMESCREEN && now - lastUpdate > updateInterval){
    lastUpdate = now; wakeDisplay();
    // Rohspannungen
    // für EC mitteln (10x)
    double sumV=0; for(int i=0;i<10;i++){ sumV+=adcVoltage(PIN_EC_ANALOG); delay(2); }
    float vEC = (float)(sumV/10.0);
    float ec  = ecFromVoltage(vEC);
    float ecSm= smoothPush(ecSamples, ecIdx, NUM_SAMPLES, ec);

    double sumP=0; for(int i=0;i<10;i++){ sumP+=adcVoltage(PIN_PH_ANALOG); delay(2); }
    float vPH = (float)(sumP/10.0);
    float ph  = phFromVoltage(vPH);
    float phSm= smoothPush(phSamples, phIdx, NUM_SAMPLES, ph);

    float waterForEC = !isnan(last_t1) ? last_t1 : (!isnan(last_t2)? last_t2 : NAN);
    float ec25 = ecTo25C(ecSm, waterForEC);
    last_ec25 = ec25; last_ph = phSm;

    drawHome(last_ec25, last_ph, last_t1, last_t2);

    // MQTT
    publishFloat(T_EC, last_ec25, 2);
    publishFloat(T_PH, last_ph, 2);
    if(!isnan(last_t1)) publishFloat(T_T1, last_t1, 1);
    if(!isnan(last_t2)) publishFloat(T_T2, last_t2, 1);
  }

  // Verlauf alle 30s
  if(now - lastHistory > 30000UL){
    lastHistory = now;
    ecHistory[histIdx] = last_ec25;
    histIdx = (histIdx+1)%HISTORY_SIZE;
    if(menuState==HISTORY_SCREEN) drawHistory();
  }

  // Menü-Timeout → Homescreen
  if(menuState!=HOMESCREEN && menuState!=CALIBRATING && !editing && (now - lastInteraction > menuTimeout)){
    menuState=HOMESCREEN; wakeDisplay();
  }

  // Info-Screen live aktualisieren
  if(menuState==INFO_SCREEN){
    drawInfo(last_ec25, last_ph, last_t1, last_t2);
  }

  // Kalibrier-Erinnerung prüfen (alle 30min)
  if(now - lastReminderCheck_ms > 1800000UL){
    lastReminderCheck_ms = now;
    unsigned long days_ms = (unsigned long)calibReminderDays * 24UL*60UL*60UL*1000UL;
    bool dueEC = (lastCalibEC_ms>0) ? (now - lastCalibEC_ms >= days_ms) : false;
    bool duePH = (lastCalibPH_ms>0) ? (now - lastCalibPH_ms >= days_ms) : false;
    if(dueEC || duePH){
      if(dueEC && duePH) mqtt.publish(T_STATUS,"Reminder: EC & pH calibration due");
      else if(dueEC)     mqtt.publish(T_STATUS,"Reminder: EC calibration due");
      else               mqtt.publish(T_STATUS,"Reminder: pH calibration due");
      // nach Meldung erneut abwarten → Timer neu setzen
      if(dueEC) lastCalibEC_ms = now;
      if(duePH) lastCalibPH_ms = now;
    }
  }
}
