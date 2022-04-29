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
#define redLedPIN               A1
#define blueLedPIN              A2


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
        if (measuredCO2 > 200) {
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
        String stringName;
        String stringValue(currentValue);
        stringValue += " ";

        categoryNameAndUnit(stringName, stringValue, currentState);

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

    static void categoryNameAndUnit(String & name, String & unit, State state) {
        switch (state) {
            case tempeature:
                name += "Tempeature";
                unit += "C";
                break;
            case humidity:
                name += "Humidity";
                unit += "%";
                break;
            case pollution:
                name += "Pollution";
                unit += "ug/m3";
                break;
            case co2:
                name += "CO2";
                unit += "ppm";
            default:
                break;
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

void lightenLED() {
    digitalWrite(redLedPIN, LOW);
    digitalWrite(blueLedPIN, LOW);
    if (co2Sensor->co2() > 1000
        || pollutionSensor->pollution() > 300) {
        digitalWrite(redLedPIN, HIGH);
    } else {
        digitalWrite(blueLedPIN, HIGH);
    }
}

void createServerResponseMeasurement(EthernetClient & client, const String & name, const String & unit, const float & value, bool validData) {
    client.println("<tr>");
    client.print("<td>");
    client.print(name);
    client.println("</td>");
    if (validData) {
        client.print("<td>");
        client.print(value);
        client.println("</td>");
        client.print("<td>");
        client.print(unit);
        client.println("</td>");
    } else {
        client.print("<td>");
        client.print("No data");
        client.println("<td>");
    }
    client.println("</tr>");
}

void createServerResponse(EthernetClient & client) {
    client.println("HTTP/1.1 200 OK");
    client.println("Content-Type: text/html");
    client.println("Connection: close");
    client.println();
    client.println("<!DOCTYPE HTML>");
    client.println("<html>");
    client.println("<h1>Air Quality Measurment Station</h1>");
    client.println("<table>");
    client.println("<tr>");
    client.println("<th>Sensor</th>");
    client.println("<th>Value</th>");
    client.println("<th>Unit</th>");
    client.println("</tr>");
    String name = "";
    String unit = "";
    Display::categoryNameAndUnit(name, unit, Display::State::tempeature);
    createServerResponseMeasurement(client, name, unit, tempeatureAndHumiditySensor->tempeature(), tempeatureAndHumiditySensor->validTempeature());
    name = "";
    unit = "";
    Display::categoryNameAndUnit(name, unit, Display::State::humidity);
    createServerResponseMeasurement(client, name, unit, tempeatureAndHumiditySensor->humidity(), tempeatureAndHumiditySensor->validHumidity());
    name = "";
    unit = "";
    Display::categoryNameAndUnit(name, unit, Display::State::pollution);
    createServerResponseMeasurement(client, name, unit, pollutionSensor->pollution(), pollutionSensor->validPollution());
    name = "";
    unit = "";
    Display::categoryNameAndUnit(name, unit, Display::State::co2);
    createServerResponseMeasurement(client, name, unit, co2Sensor->co2(), co2Sensor->validCO2());
    client.println("</table>");
    client.println("</html>");
}

void measureAll() {
    display->measuring = true;
    display->draw();

    tempeatureAndHumiditySensor->measure();
    pollutionSensor->measure();
    co2Sensor->measure();

    Serial.println("Measurement:: Measuring");

    display->measuring = false;
    passToDisplay();

    lightenLED();
}

void setup() {
    Serial.begin(9600);

    tempeatureAndHumiditySensor = new TempeatureHumiditySensor();
    pollutionSensor = new PollutionSensor();
    co2Sensor = new CO2Sensor();

    display = new Display();
    display->draw();

    Ethernet.begin(mac, ip);
    if (Ethernet.hardwareStatus() == EthernetNoHardware) {
        Serial.println("Server:: Error: Hardware not connected");
    }
    if (Ethernet.linkStatus() == LinkOFF) {
        Serial.println("Server:: Cable not plugged in.");
    }
    server.begin();
    Serial.print("Server:: Starting server at ");
    Serial.println(Ethernet.localIP());

    pinMode(buttonPIN, INPUT_PULLUP);

    pinMode(redLedPIN, OUTPUT);
    pinMode(blueLedPIN, OUTPUT);

    measureAll();
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
        Serial.println("Server:: New connection");
        bool blankRequestLine = true;
        while (client.connected()) {
            if (client.available()) {
                char c = client.read();

                if (c == '\n' && blankRequestLine) {
                    createServerResponse(client);
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
        Serial.println("Server:: Ended connection");
    }
}
