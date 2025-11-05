#include <Arduino.h>
#include <HX711.h>
#include <Preferences.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <math.h>


#define HX_DT   25 //32
#define HX_SCK  26 //33


#define BATT_ADC_PIN   34
#define BATT_CHARGING  -1     


static float CAL_FACTOR_A = 420.0f;       //Load Cell A
static float CAL_FACTOR_B = 420.0f;       // Load Cell B

static const float   MAX_G     = 10000.0f;   
static const uint32_t SEND_MS  = 300;       
static const float   QUIET_EPS = 20.0f;     
static const uint32_t QUIET_MS = 2500;       

/* =========================== */
#define DIV_FACTOR     2.0f   // 100k + 100k ⇒ ×2
#define SAMPLES        16
static float emaV = 0.0f;
static const float alpha = 0.20f;  // EMA
/* =========================== */
static const char* BLE_NAME = "ProGearBag";
#define UUID_SVC "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define UUID_RX  "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"   // write
#define UUID_TX  "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"   // notify

HX711        scale;          
Preferences  prefs;

BLECharacteristic* txCh = nullptr;
BLECharacteristic* rxCh = nullptr;
String rxBuf;

float baselineA = 0.0f;
float baselineB = 0.0f;
float lastG     = -1.0f;
float expectedG = 0.0f;

uint32_t lastSend   = 0;
uint32_t quietStart = 0;
bool inQuiet = false, sessionFinalSent = false;

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

//Battery Read 
static int readBatteryPct() {
  if (BATT_ADC_PIN < 0) return -1;
  analogReadResolution(12);
  uint32_t acc = 0;
  for (int i=0;i<SAMPLES;i++){ acc += analogRead(BATT_ADC_PIN); delay(2); }
  int raw = acc / SAMPLES;

  float vadc = (raw / 4095.0f) * 3.3f;
  float v    = vadc * DIV_FACTOR;                


  float pctf = (v - 3.40f) / (4.20f - 3.40f) * 100.0f;
  return (int)round(constrain(pctf, 0.0f, 100.0f));
}
static int readCharging() {
  if (BATT_CHARGING < 0) return 0;
  pinMode(BATT_CHARGING, INPUT_PULLUP);
  return digitalRead(BATT_CHARGING) ? 1 : 0;
}
static void sendBatteryNow(){
  int p = readBatteryPct();
  if (p >= 0) sendBattery(p, readCharging());
}


static float readGramsA() {
  //Choose Load Cell A - Gain 128
  scale.set_gain(128);
  if (!scale.is_ready()) return NAN;
  (void)scale.read();                 // رمية أولى بعد التحويل
  if (!scale.is_ready()) return NAN;
  scale.set_scale(CAL_FACTOR_A);
  return scale.get_units(3) - baselineA; // متوسط 3 قراءات
}

static float readGramsB() {
  // Choose Load Cell B - Gain 32
  scale.set_gain(32);
  if (!scale.is_ready()) return NAN;
  (void)scale.read();                 // رمية أولى بعد التحويل
  if (!scale.is_ready()) return NAN;
  scale.set_scale(CAL_FACTOR_B);
  return scale.get_units(3) - baselineB; // متوسط 3 قراءات
}

// tare both load cells
static void tareBoth() {
  // Load Cell A
  scale.set_gain(128);
  if (scale.is_ready()) { scale.set_scale(CAL_FACTOR_A); scale.tare(15); }
  baselineA = 0.0f;

  // Load Cell B
  scale.set_gain(32);
  if (scale.is_ready()) { scale.set_scale(CAL_FACTOR_B); scale.tare(15); }
  baselineB = 0.0f;
}
//calculate total weight 
float readTotalGrams() {
  float gA = readGramsA();
  float gB = readGramsB();

  if (isnan(gA) || isnan(gB)) return -1.0f;

  float total = gA + gB;
  if (total < 0) total = 0;
  return total;
}

/* ===================== BLE RX callbacks ================= */
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
        sendLine("PONG", "{}");
      }
      else if (U == "RESET_WEIGHT") {
        tareBoth();
        lastG = -1.0f;
        inQuiet = false; sessionFinalSent = false;
        sendStatus("tare_done");
        sendStatus("baseline_reset");
      }
      else if (U.startsWith("SET_EXPECTED_WEIGHT:")) {
        int colon = line.indexOf(':');
        if (colon > 0) {
          float val = line.substring(colon+1).toFloat();
          saveExpected(val);
        }
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

/* ========================== setup ======================= */
void setup() {
  Serial.begin(115200);
  delay(100);

  // NVS
  prefs.begin("progear", false);
  float savedCalA = prefs.getFloat("calA", NAN);
  float savedCalB = prefs.getFloat("calB", NAN);
  if (!isnan(savedCalA)) CAL_FACTOR_A = savedCalA;
  if (!isnan(savedCalB)) CAL_FACTOR_B = savedCalB;
  expectedG = prefs.getFloat("exp_g", 0.0f);

  scale.begin(HX_DT, HX_SCK);
  scale.set_gain(128); // A
  if (!scale.is_ready()) {
    sendStatus("scale_not_ready");
  } else {
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

/* =========================== loop ======================= */
void loop() {
  uint32_t now = millis();

  if (now - lastSend >= SEND_MS) {
    float g = readTotalGrams();

    if (g < 0) {
      sendStatus("scale_not_ready");
    } else {
      if (g > MAX_G) {
        sendError("over_capacity");
      } else {
        sendWeight(g);  // WEIGHT_DATA:{ "g": ... }
      }

      bool stable = (lastG < 0) ? false : fabsf(g - lastG) <= QUIET_EPS;
      if (stable) {
        if (!inQuiet) { inQuiet = true; quietStart = now; sessionFinalSent = false; }
        else if (!sessionFinalSent && (now - quietStart) >= QUIET_MS) {
          sendSessionFinal(g);
          sessionFinalSent = true;
        }
      } else {
        inQuiet = false; sessionFinalSent = false; quietStart = now;
      }
      lastG = g;
    }
    lastSend = now;
  }

  static uint32_t lastBat = 0;
  if (BATT_ADC_PIN >= 0 && now - lastBat > 10000) {
    sendBatteryNow();
    lastBat = now;
  }
}