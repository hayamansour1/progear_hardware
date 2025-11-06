#include <Arduino.h>
#include <HX711.h>
#include <Preferences.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <math.h>

#define HX_DT   25           
#define HX_SCK  26          

#define BATT_ADC_PIN   34
#define BATT_CHARGING  -1    
#define DIV_FACTOR     2.0f  
#define SAMPLES_ADC    16

static float CAL_A = 210.0f;     // counts per gram cell A (Gain=128)
static float CAL_B = 210.0f;     // counts per gram cell B (Gain=32)
#define SAMPLES_TARE   20        
#define SAMPLES_READ   10        
static const float   MAX_G     = 10000.0f;
static const uint32_t SEND_MS  = 300;     
static const float   QUIET_EPS = 20.0f;   
static const uint32_t QUIET_MS = 2500;    
static const uint32_t GAIN_SETTLE_MS = 8; 

static const uint32_t COOLDOWN_MS = 1500;
uint32_t lastScaleNR = 0, lastOverCap = 0;

static const char* BLE_NAME = "ProGearBag";
#define UUID_SVC "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define UUID_RX  "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"   // write
#define UUID_TX  "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"   // notify

HX711        scale;
Preferences  prefs;

BLECharacteristic* txCh = nullptr;
BLECharacteristic* rxCh = nullptr;
String rxBuf;

float   lastTotal = -1.0f;
float   expectedG = 0.0f;
uint32_t lastSend   = 0;
uint32_t quietStart = 0;
bool    inQuiet = false, sessionFinalSent = false;

static void sendLine(const String& tag, const String& json) {
  String s = tag + ":" + json + "\n";
  Serial.print(s);
  if (txCh) { txCh->setValue((uint8_t*)s.c_str(), s.length()); txCh->notify(); }
}
static void sendStatus(const String& st)            { sendLine("STATUS_UPDATE", String("{\"state\":\"")+st+"\"}"); }
static void sendStatusExpected(float g)             { sendLine("STATUS_UPDATE", String("{\"expected_weight_g\":")+String(g,0)+"}"); }
static void sendError (const String& msg)           { sendLine("ERROR",          String("{\"msg\":\"")+msg+"\"}"); }
static void sendWeight(float g)                     { sendLine("WEIGHT_DATA",    String("{\"g\":")+String(g,1)+"}"); }
static void sendSessionFinal(float g)               { sendLine("SESSION_FINAL",  String("{\"g\":")+String(g,1)+"}"); }
static void sendBattery(int pct, int ch)            { sendLine("BATTERY",        String("{\"percent\":")+pct+",\"chg\":"+ch+"}"); }

static void saveExpected(float g){
  expectedG = g;
  prefs.putFloat("exp_g", expectedG);
  sendStatusExpected(expectedG);
}

static int readCharging() {
  if (BATT_CHARGING < 0) return 0;
  pinMode(BATT_CHARGING, INPUT_PULLUP);
  return digitalRead(BATT_CHARGING) ? 1 : 0;
}
static int readBatteryPct() {
  if (BATT_ADC_PIN < 0) return -1;
  analogReadResolution(12);
  uint32_t acc = 0;
  for (int i=0;i<SAMPLES_ADC;i++){ acc += analogRead(BATT_ADC_PIN); delay(2); }
  int raw = acc / SAMPLES_ADC;
  float vadc = (raw / 4095.0f) * 3.3f;   // ADC @ 3.3V
  float v    = vadc * DIV_FACTOR;        
  float pctf = (v - 3.40f) / (4.20f - 3.40f) * 100.0f;
  return (int)round(constrain(pctf, 0.0f, 100.0f));
}
static void sendBatteryNow(){
  int p = readBatteryPct();
  if (p >= 0) sendBattery(p, readCharging());
}

static long readCounts(byte gain, uint8_t samples){
  scale.set_gain(gain);
  delay(GAIN_SETTLE_MS);       
  if (!waitReady(80)) return LONG_MIN;
  (void)scale.read();    
  if (!waitReady(80)) return LONG_MIN;    
  return scale.get_value(samples); 
}

static void tareBoth(){
  // A (gain 128)
  scale.set_gain(128);
  delay(GAIN_SETTLE_MS);
  scale.tare(SAMPLES_TARE);

  // B (gain 32)
  scale.set_gain(32);
  delay(GAIN_SETTLE_MS);
  scale.tare(SAMPLES_TARE);
}

static float readTotalGrams(){
  if (!scale.is_ready()) return -1.0f;

  long netA = readCounts(128, SAMPLES_READ);   // A
  long netB = readCounts(32 , SAMPLES_READ);   // B
  if (netA -- LONG_MIN || netB -- LONG_MIN) return -1.0f;
  float gA  = (float)netA / CAL_A;
  float gB  = (float)netB / CAL_B;
  float total = gA + gB;
  if (total < 0) total = 0;
  return total;
}

/* ---------------------- BLE RX ------------------------ */
class RxCB : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* c) override {
    String v = c->getValue();
    if (v.length() == 0) return;
    rxBuf += v;

    int idx;
    while ((idx = rxBuf.indexOf('\n')) >= 0) {
      String line = rxBuf.substring(0, idx);
      rxBuf.remove(0, idx+1);
      line.trim();
      String U = line; U.toUpperCase();

      if (U == "PING") {
        sendStatus("pong");
      }
      else if (U == "RESET_WEIGHT") {
        tareBoth();
        lastTotal = -1.0f;
        inQuiet = false; sessionFinalSent = false;
        sendStatus("tare_done");
        sendStatus("baseline_reset");
      }
      else if (U.startsWith("SET_EXPECTED_WEIGHT:")) {
        int colon = line.indexOf(':');
        if (colon > 0) saveExpected(line.substring(colon+1).toFloat());
      }
      else if (U == "GET_EXPECTED_WEIGHT") {
        sendStatusExpected(expectedG);
      }
      else if (U == "GET_BAT") {
        sendBatteryNow();
      }
    }
  }
};

void setup() {
  Serial.begin(115200);
  delay(100);

  // NVS
  prefs.begin("progear", false);
  expectedG = prefs.getFloat("exp_g", 0.0f);

  // HX711
  scale.begin(HX_DT, HX_SCK);
  if (!scale.is_ready()) sendStatus("scale_not_ready");
  else {
    tareBoth();
    sendStatus("scale_ready");
    sendStatus("tare_done");
  }

  // BLE
  BLEDevice::init(BLE_NAME);
  BLEServer* srv = BLEDevice::createServer();
  BLEService* svc = srv->createService(UUID_SVC);

  txCh = svc->createCharacteristic(UUID_TX, BLECharacteristic::PROPERTY_NOTIFY);
  rxCh = svc->createCharacteristic(UUID_RX, BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
  rxCh->setCallbacks(new RxCB());

  svc->start();
  BLEAdvertising* adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(UUID_SVC);
  adv->setScanResponse(true);
  adv->start();

  sendStatus("ble_advertising");
  if (expectedG > 0) sendStatusExpected(expectedG);

  sendBatteryNow();
}

/* ----------------------- loop ------------------------- */
void loop() {
  uint32_t now = millis();

  if (now - lastSend >= SEND_MS) {
    float total = readTotalGrams();

    if (total < 0) {
      if (now - lastScaleNR >= COOLDOWN_MS) {
        sendStatus("scale_not_ready");
        lastScaleNR = now;
      }
    } else {
      if (total > MAX_G) {
        if (now - lastOverCap >= COOLDOWN_MS) {
          sendError("over_capacity");
          lastOverCap = now;
        }
      } else {
        // نرسل TOTAL فقط
        sendWeight(total);
      }

      bool stable = (lastTotal < 0) ? false : fabsf(total - lastTotal) <= QUIET_EPS;
      if (stable) {
        if (!inQuiet) { inQuiet = true; quietStart = now; sessionFinalSent = false; }
        else if (!sessionFinalSent && (now - quietStart) >= QUIET_MS) {
          sendSessionFinal(total);
          sessionFinalSent = true;
        }
      } else {
        inQuiet = false; sessionFinalSent = false; quietStart = now;
      }
      lastTotal = total;
    }

    lastSend = now;
  }

  // بطارية كل 10 ثواني
  static uint32_t lastBat = 0;
  if (BATT_ADC_PIN >= 0 && now - lastBat > 10000) {
    sendBatteryNow();
    lastBat = now;
  }
}