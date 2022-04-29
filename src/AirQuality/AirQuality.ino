#include <MHZ19.h>
#include <Ethernet.h>
#include <U8glib.h>
#include <DHT.h>
#include <DHT_U.h>


byte mac [] = {
    0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED
};
IPAddress ip(192, 168, 0, 102);


#define tempeatureDigitalPIN    5
#define pollutionAnalogPIN      A0
#define pollutionDigitalPIN     3
#define co2RxPIN                2
#define co2TxPIN                4
#define buttonPIN               6


class TempeatureHumiditySensor {
private:
    DHT librarySensor;

    float measuredTempeature;
    bool isTempeatureValid;

    float measuredHumidity;
    bool isHumidityValid;

public:
    TempeatureHumiditySensor() : librarySensor(tempeatureDigitalPIN, DHT22), measuredTempeature(0.0), isTempeatureValid(false), measuredHumidity(0.0), isHumidityValid(false) {
        librarySensor.begin();
    }

    void measure() {
        measuredTempeature = librarySensor.readTemperature();
        measuredHumidity = librarySensor.readHumidity();

        isTempeatureValid = !isnan(measuredTempeature);
        isHumidityValid = !isnan(measuredHumidity);
    }

    float tempeature() {
        return measuredTempeature;
    }

    bool validTempeature() {
        return isTempeatureValid;
    }

    float humidity() {
        return measuredHumidity;
    }

    bool validHumidity() {
        return isHumidityValid;
    }
};

class PollutionSensor {

    float measuredPollution;
    bool isPollutionValid;

public:
    PollutionSensor() : measuredPollution(0.0), isPollutionValid(false) {
        pinMode(pollutionDigitalPIN, OUTPUT);
    }

    void measure() {
        float measuredPollVoltage = 0;
        digitalWrite(pollutionDigitalPIN, LOW);
        delayMicroseconds(280);
        measuredPollVoltage = analogRead(pollutionAnalogPIN);
        delayMicroseconds(40);
        digitalWrite(pollutionDigitalPIN, HIGH);
        delayMicroseconds(9800);
        float measuredPollVoltageToDigital = measuredPollVoltage * (5.0 / 1024.0);
        measuredPollution = (0.170 * measuredPollVoltageToDigital - 0.11) * 1000.0;

        if (measuredPollution > 0) {
            isPollutionValid = true;
        } else {
            isPollutionValid = false;
        }
    }

    float pollution() {
        return measuredPollution;
    }

    bool validPollution() {
        return isPollutionValid;
    }
};

class CO2Sensor {

    float measuredCO2;
    bool isCO2Valid;

    MHZ19 * mhz19uart;

public:
    CO2Sensor() : mhz19uart(new MHZ19(co2RxPIN, co2TxPIN)) {
        mhz19uart->begin(co2RxPIN, co2TxPIN);
    }

    void measure() {
        measurement_t co2measured = mhz19uart->getMeasurement();
        measuredCO2 = map(co2measured.co2_ppm, 0, 5000, 0, 2000);
        if (measuredCO2 > 0) {
            isCO2Valid = true;
        } else {
            isCO2Valid = false;
        }
    }

    float co2() {
        return measuredCO2;
    }

    float validCO2() {
        return isCO2Valid;
    }
};

class Display {
private:
    U8GLIB_SSD1306_128X64 libraryDisplay;

    void redrawFail() {
        libraryDisplay.setPrintPos(64 - (libraryDisplay.getStrWidth("no data") / 2), 30);
        libraryDisplay.print("no data");
    }

    void redrawMeasuring() {
        libraryDisplay.setPrintPos(64 - (libraryDisplay.getStrWidth("measuring...") / 2), 10);
        libraryDisplay.print("measuring...");
    }

    void redrawMesurement() {
        String stringValue(currentValue);
        String stringName;

        switch (currentState) {
            case tempeature:
                stringName = "Tempeature";
                stringValue += " C";
                break;
            case humidity:
                stringName = "Humidity";
                stringValue += " %";
                break;
            case pollution:
                stringName = "Pollution";
                stringValue += " ug/m3";
                break;
            case co2:
                stringName = "CO2";
                stringValue += " ppm";
            default:
                break;
        }

        libraryDisplay.setPrintPos(64 - (libraryDisplay.getStrWidth(stringName.c_str()) / 2), 10);
        libraryDisplay.print(stringName.c_str());
        libraryDisplay.setPrintPos(64 - (libraryDisplay.getStrWidth(stringValue.c_str()) / 2), 30);
        libraryDisplay.print(stringValue.c_str());
    }

public:
    enum State {
        tempeature = 0,
        humidity = 1,
        pollution = 2,
        co2 = 3
    };

    bool measuring;
    bool noData;
    enum State currentState;
    float currentValue;

    Display() : libraryDisplay(U8G_I2C_OPT_NONE), measuring(false), noData(true), currentState(tempeature), currentValue(0.0) {
        libraryDisplay.setFont(u8g_font_unifont);
    }

    void draw() {
        libraryDisplay.firstPage();
        do {
            if (measuring) {
                redrawMeasuring();
            } else if (noData) {
                redrawFail();
            } else {
                redrawMesurement();
            }
        } while (libraryDisplay.nextPage());
    }

    void raiseState() {
        if (currentState == 3) {
            currentState = 0;
        } else {
            currentState = currentState + 1;
        }
    }
};

TempeatureHumiditySensor * tempeatureAndHumiditySensor;
PollutionSensor * pollutionSensor;
CO2Sensor * co2Sensor;

Display * display;

EthernetServer server(80);

void passToDisplay() {
    switch (display->currentState) {
        case Display::State::tempeature:
            display->currentValue = tempeatureAndHumiditySensor->tempeature();
            display->noData = !tempeatureAndHumiditySensor->validTempeature();
            break;
        case Display::State::humidity:
            display->currentValue = tempeatureAndHumiditySensor->humidity();
            display->noData = !tempeatureAndHumiditySensor->validHumidity();
            break;
        case Display::State::pollution:
            display->currentValue = pollutionSensor->pollution();
            display->noData = !pollutionSensor->validPollution();
            break;
        case Display::State::co2:
            display->currentValue = co2Sensor->co2();
            display->noData = !co2Sensor->validCO2();
        default:
            break;
    }
    display->draw();
}

void measureAll() {
    display->measuring = true;
    display->draw();

    tempeatureAndHumiditySensor->measure();
    pollutionSensor->measure();
    co2Sensor->measure();

    if (tempeatureAndHumiditySensor->validTempeature()) {
        Serial.print("Measurement:: tempeature: ");
        Serial.print(tempeatureAndHumiditySensor->tempeature());
        Serial.println(" C");
    }

    if (tempeatureAndHumiditySensor->validHumidity()) {
        Serial.print("Measurement:: humidity: ");
        Serial.print(tempeatureAndHumiditySensor->humidity());
        Serial.println(" %");
    }

    if (pollutionSensor->validPollution()) {
        Serial.print("Measurement:: pollution: ");
        Serial.print(pollutionSensor->pollution());
        Serial.println(" ug/m3");
    }

    if (co2Sensor->validCO2()) {
        Serial.print("Measurement:: co2: ");
        Serial.print(co2Sensor->co2());
        Serial.println(" ppm");
    }

    display->measuring = false;
    passToDisplay();
}

void setup() {
    Serial.begin(9600);
    while (!Serial) {
        ;
    }

    tempeatureAndHumiditySensor = new TempeatureHumiditySensor();
    pollutionSensor = new PollutionSensor();
    co2Sensor = new CO2Sensor();

    display = new Display();
    display->draw();

    Ethernet.begin(mac, ip);
    if (Ethernet.hardwareStatus() == EthernetNoHardware) {
        Serial.println("Server:: Error: Ethernet hardware not connected");
    }
    if (Ethernet.linkStatus() == LinkOFF) {
        Serial.println("Server:: Ethernet cable not plugged in.");
    }
    server.begin();
    Serial.print("Server:: Starting server at ");
    Serial.println(Ethernet.localIP());

    pinMode(buttonPIN, INPUT_PULLUP);
}

unsigned long lastActionTime = 0;
unsigned long lastDebounceTime = 0;

void loop() {

    if (millis() - lastActionTime > 7000) {
      measureAll();
      lastActionTime = millis();
    }

    if ((millis() - lastDebounceTime) > 200) {
      if (digitalRead(buttonPIN) == LOW && !display->measuring) {
        display->raiseState();
        passToDisplay();
        lastDebounceTime = millis();
      }
    }

    EthernetClient client = server.available();
    if (client && !display->measuring) {
        Serial.println("Server:: New server connection");
        bool blankRequestLine = true;
        while (client.connected()) {
            if (client.available()) {
                char c = client.read();

                if (c == '\n' && blankRequestLine) {
                    client.println("HTTP/1.1 200 OK");
                    client.println("Content-Type: text/html");
                    client.println("Connection: close");
                    client.println();
                    client.println("<!DOCTYPE HTML>");
                    client.println("<html>");
                    client.println("hello world!");
                    client.println("</html>");
                    break;
                }
                if (c == '\n') {
                    blankRequestLine = true;
                } else if (c != '\r') {
                    blankRequestLine = false;
                }
            }
        }
        delay(1);
        client.stop();
        Serial.println("Server:: Ended server connection");
    }
}
