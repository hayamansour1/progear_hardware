#include <HX711.h>
#include "BluetoothSerial.h"

#define DT 25  // HX711 data pin
#define SCK 26 // HX711 clock pin

HX711 scale;
BluetoothSerial SerialBT;

float calibration_factor = 420.0; 
float baseWeight = 0;

void setup() {
  Serial.begin(115200);
  SerialBT.begin("ProGearBag"); // Bluetooth name

  scale.begin(DT, SCK);
  scale.set_scale(calibration_factor);

  if (!scale.is_ready()) {
    Serial.println("HX711 not found. Check wiring.");
    while (true); // Stop if HX711 isn't connected
  }

  scale.tare(); // Reset scale to 0
  delay(2000);  
  baseWeight = scale.get_units();

  Serial.println("ProGear Smart Bag started");
  SerialBT.println("ProGear Smart Bag started");
}

void loop() {
  if (scale.is_ready()) {
    float weight = scale.get_units() - baseWeight;

    if (weight < 0) weight = 0;

    String weightStr = String(weight, 2) + " g";

    Serial.println(weightStr);
    SerialBT.println(weightStr);
  } else {
    Serial.println("HX711 not found. Check wiring.");
    SerialBT.println("HX711 not found. Check wiring.");
  }

  delay(1000); 
}
