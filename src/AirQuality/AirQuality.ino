#include <Ethernet.h>
#include <U8glib.h>
#include <DHT.h>
#include <DHT_U.h>


#define tempeatureDigitalPIN 3
#define pollutionAnalogPIN A0
#define pollutionDigitalPIN 5


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
        measuredPollution = (0.17 * measuredPollVoltageToDigital) * 1000;

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
        pollution = 2
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
        if (currentState == 2) {
            currentState = 0;
        } else {
            currentState = currentState + 1;
        }
    }
};

TempeatureHumiditySensor * tempeatureAndHumiditySensor;
PollutionSensor * pollutionSensor;

Display * display;

void passToDisplay() {
    switch (display->currentState) {
        case Display::State::tempeature:
            display->currentValue = tempeatureAndHumiditySensor->tempeature();
            display->noData = tempeatureAndHumiditySensor->validTempeature();
            break;
        case Display::State::humidity:
            display->currentValue = tempeatureAndHumiditySensor->humidity();
            display->noData = tempeatureAndHumiditySensor->validHumidity();
            break;
        case Display::State::pollution:
            display->currentValue = pollutionSensor->pollution();
            display->noData = pollutionSensor->validPollution();
            break;
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

    if (tempeatureAndHumiditySensor->validTempeature()) {
        Serial.print("tempeature: ");
        Serial.println(tempeatureAndHumiditySensor->tempeature());
    }

    if (tempeatureAndHumiditySensor->validHumidity()) {
        Serial.print("humidity: ");
        Serial.println(tempeatureAndHumiditySensor->humidity());
    }

    if (pollutionSensor->validPollution()) {
        Serial.print("pollution: ");
        Serial.println(pollutionSensor->pollution());
    }

    display->measuring = false;
    passToDisplay();
}

void setup() {
    Serial.begin(9600);

    tempeatureAndHumiditySensor = new TempeatureHumiditySensor();
    pollutionSensor = new PollutionSensor();

    display = new Display();
}

void loop() {
    measureAll();
    delay(10000);
}
