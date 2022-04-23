#include <DHT.h>
#include <DHT_U.h>

#define tempPIN 3
#define dhtType DHT22

DHT tempSensor(tempPIN, dhtType);

void setup() {
  Serial.begin(9600);
  tempSensor.begin();
}

void loop() {
  float t = tempSensor.readTemperature();
  float h = tempSensor.readHumidity();

  if (!isnan(t) && !isnan(h)) {
    Serial.print("tempeature: ");
    Serial.print(t);
    Serial.print("; ");
    Serial.print("humidity: ");
    Serial.println(h);
  } else {
    Serial.println("tmp: err");
  }

  delay(3000);
}
