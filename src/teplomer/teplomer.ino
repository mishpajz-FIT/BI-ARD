#include <U8glib.h>
#include <DHT.h>
#include <DHT_U.h>


#define tempPIN 3
#define dhtType DHT22

#define pollutionAnalogPIN A0
#define pollutionDigitalPIN 5

DHT tempSensor(tempPIN, dhtType);

U8GLIB_SSD1306_128X64 display(U8G_I2C_OPT_NONE);
long int displayRefresh = 0;

void setup() {
  Serial.begin(9600);
  tempSensor.begin();

  pinMode(pollutionDigitalPIN, OUTPUT);
}

void loop() {
  /* tempeature */
  float measuredTemp = tempSensor.readTemperature();
  float measuredHumi = tempSensor.readHumidity();

  if (!isnan(measuredTemp) && !isnan(measuredHumi)) {
    Serial.print("tempeature: ");
    Serial.print(measuredTemp);
    Serial.print("; ");
    Serial.print("humidity: ");
    Serial.println(measuredHumi);
  } else {
    Serial.println("tmp || hmd: err");
  }


  /* pollution */
  float measuredPollVoltage = 0;
  digitalWrite(pollutionDigitalPIN, LOW);
  delayMicroseconds(280);
  measuredPollVoltage = analogRead(pollutionAnalogPIN);
  delayMicroseconds(40);
  digitalWrite(pollutionDigitalPIN, HIGH);
  delayMicroseconds(9800);
  float measuredPollVoltageToDigital = measuredPollVoltage * (5.0 / 1024.0);

  float measuredPollution = (0.17 * measuredPollVoltageToDigital) * 1000;
  Serial.print(measuredPollVoltageToDigital);
  Serial.print(" V; ");
  Serial.print(" -> pollution: ");
  Serial.print(measuredPollution);
  Serial.println(" ug/m3");

  if (millis()-displayRefresh > 100) {
    display.firstPage(); 
    do {
      display.setFont(u8g_font_unifont);
      display.setPrintPos(0, 10);
      display.print(measuredTemp);
    } while (display.nextPage());

    displayRefresh = millis();
  }
  
  delay(3000);
}
