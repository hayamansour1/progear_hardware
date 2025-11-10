#include <Arduino.h>
#include <HX711.h>
#include <Preferences.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <math.h>

// ===== Pins =====
#define HX_DT   32
#define HX_SCK  33
#define BATT_ADC_PIN   34
#define BATT_CHARGING  -1     

// ===== Calibration (افتراضي) =====
static float CAL_A = 210.0f;   // Cell A (Gain=128)
static float CAL_B = 210.0f;   // Cell B (Gain=32)

// ===== Settings =====
static const float   MAX_G     = 10000.0f;   
static const uint32_t SEND_MS  = 500;       
static const float   QUIET_EPS = 20.0f;     
static const uint32_t QUIET_MS = 2500;       

// ===== Battery =====
#define DIV_FACTOR     2.0f
#define SAMPLES        16
static float emaV = NAN;
static const float alpha = 0.20f;
static int lastBatPct = -1;

// ===== BLE =====
static const char* BLE_NAME = "ProGearBag";
#define UUID_SVC "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define UUID_RX  "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define UUID_TX  "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

HX711        scale;          
Preferences  prefs;

BLECharacteristic* txCh = nullptr;
BLECharacteristic* rxCh = nullptr;
String rxBuf;

// ===== tare offsets =====
long offsetA = 0;
long offsetB = 0;

// ===== state =====
float lastG     = -1.0f;
uint32_t lastSend   = 0;
uint32_t quietStart = 0;
bool inQuiet = false, sessionFinalSent = false;
static uint32_t lastErrorMs = 0;

// ===== Utilities =====
static void sendLine(const String& tag, const String& json) {
  String s = tag + ":" + json + "\n";
  Serial.print(s);
  if (txCh) { txCh->setValue((uint8_t*)s.c_str(), s.length()); txCh->notify(); }
}
static void sendWeight(float g)   { sendLine("WEIGHT_DATA", String("{\"g\":")+String(g,1)+"}"); }
static void sendSessionFinal(float g) { sendLine("SESSION_FINAL", String("{\"g\":")+String(g,1)+"}"); }
static void sendError(const String& msg) { sendLine("ERROR", String("{\"msg\":\"")+msg+"\"}"); }
static void sendStatus(const String& st) { sendLine("STATUS_UPDATE", String("{\"state\":\"")+st+"\"}"); }
static void sendBatteryMsg(int pct, int ch) { sendLine("BATTERY", String("{\"percent\":")+pct+",\"chg\":"+ch+"}"); }

// ===== Battery Read with EMA =====
static int readBatteryPct() {
  if (BATT_ADC_PIN < 0) return -1;
  analogReadResolution(12);
  uint32_t acc = 0;
  for (int i=0;i<SAMPLES;i++){ acc += analogRead(BATT_ADC_PIN); delay(2); }
  int raw = acc / SAMPLES;

  float vadc = (raw / 4095.0f) * 3.3f;
  float v    = vadc * DIV_FACTOR;                

  if (isnan(emaV)) emaV = v;
  else emaV = alpha * v + (1.0f - alpha) * emaV;

  float pctf = (emaV - 3.40f) / (4.20f - 3.40f) * 100.0f;
  return (int)round(constrain(pctf, 0.0f, 100.0f));
}
static int readCharging() {
  if (BATT_CHARGING < 0) return 0;
  pinMode(BATT_CHARGING, INPUT_PULLUP);
  return digitalRead(BATT_CHARGING) ? 1 : 0;
}
static void checkBatteryAlert(){
  int p = readBatteryPct();
  if (p >= 0) {
    if (lastBatPct < 0) {
      sendBatteryMsg(p, readCharging()); // أول مرة
    }
    else if (p <= 20 && lastBatPct > 20) {
      sendBatteryMsg(p, readCharging()); // تنبيه نزول تحت 20%
    }
    lastBatPct = p;
  }
}

// ===== HX711 Dual Channel Read =====
long readChannelRaw(byte gain, uint8_t samples) {
  scale.set_gain(gain);
  delay(80); // تأخير أطول لضمان استقرار القراءة
  return scale.read_average(samples);
}
void tareBoth() {
  offsetA = readChannelRaw(128, 20);
  offsetB = readChannelRaw(32,  20);
}
float rawA_to_grams(long rawA) {
  long net = rawA - offsetA;
  return (float)net / CAL_A;
}
float rawB_to_grams(long rawB) {
  long net = rawB - offsetB;
  return (float)net / CAL_B;
}
float readWeightA_g() {
  long rawA = readChannelRaw(128, 10);
  return rawA_to_grams(rawA);
}
float readWeightB_g() {
  long rawB = readChannelRaw(32, 10);
  return rawB_to_grams(rawB);
}
float readTotalGrams() {
  float a_g = readWeightA_g();
  float b_g = readWeightB_g();
  float total = a_g + b_g;

  // فلتر بسيط: أي قراءة أقل من ±2g تعتبر صفر (للتخلص من الضجيج)
  if (fabs(total) < 2.0f) {
    total = 0.0f;
  }

  return total;
}

// ===== BLE RX callbacks =====
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

      if (U == "RESET_WEIGHT") {
        tareBoth();
        lastG = -1.0f;
        inQuiet = false; sessionFinalSent = false;
        sendStatus("baseline_reset");
      }
      else if (U == "PING") {
        sendStatus("pong");
      }
      else if (U == "GET_BAT") {
        checkBatteryAlert();
      }
    }
  }
};

// ===== setup =====
void setup() {
  Serial.begin(115200);
  delay(100);

  scale.begin(HX_DT, HX_SCK);
  tareBoth(); // تصفير عند التشغيل

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

  checkBatteryAlert(); // أول إرسال للبطارية
}

// ===== loop =====
void loop() {
  uint32_t now = millis();

  if (now - lastSend >= SEND_MS) {
    float g = readTotalGrams();

    if (g > MAX_G) {
      if (now - lastErrorMs > 5000) {
        sendError("over_capacity");
        lastErrorMs = now;
      }
    } else {
      if (lastG < 0 || fabsf(g - lastG) >= 1.0f) {
        sendWeight(g);
      }

      bool stable = (lastG >= 0) && (fabsf(g - lastG) <= QUIET_EPS);
      if (stable) {
        if (!inQuiet) {
          inQuiet = true;
          quietStart = now;
          sessionFinalSent = false;
        } else if (!sessionFinalSent && (now - quietStart) >= QUIET_MS) {
          sendSessionFinal(g);
          sessionFinalSent = true;
        }
      } else {
        inQuiet = false;
        sessionFinalSent = false;
        quietStart = now;
      }

      lastG = g;
    }

    lastSend = now;
  }

  // تحديث البطارية
  checkBatteryAlert();
}