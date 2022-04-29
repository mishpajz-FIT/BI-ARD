//SECTION: Header
/****************************************
 * @file AirQuality.ino
 * @author Michal Dobes
 * @version 1.0
 * @date 2022-04-30
 * @brief Air Quality Measurement Station
 *
 * @copyright Copyright (c) 2022
 *
 ****************************************/

/****************************************
 * Libraries
 ****************************************/
#include <MHZ19.h>
#include <Ethernet.h>
#include <U8glib.h>
#include <DHT.h>
#include <DHT_U.h>

/****************************************
 * Ethernet settings
 ****************************************/
byte mac [] = {
    0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED
}; //< MAC address of ethernet module

IPAddress ip(192, 168, 0, 102); //< Assigned static IP address


/****************************************
 * Pins
 ****************************************/
#define tempeatureDigitalPIN    5
#define pollutionAnalogPIN      A0
#define pollutionDigitalPIN     3
#define co2RxPIN                4
#define co2TxPIN                2
#define buttonPIN               6
#define redLedPIN               A1
#define blueLedPIN              A2

/****************************************
 * Constants
 ****************************************/
#define measureRate             7000

//!SECTION: Header

//SECTION: Implementation
//SECTION: Sensors
/**
 * @brief DHT-22 tempeature and humidity sensor
 *
 */
class TempeatureHumiditySensor {
private:
    DHT librarySensor; //< Sensor library object

    float measuredTempeature;
    bool isTempeatureValid;

    float measuredHumidity;
    bool isHumidityValid;

public:
    TempeatureHumiditySensor() : librarySensor(tempeatureDigitalPIN, DHT22), measuredTempeature(0.0), isTempeatureValid(false), measuredHumidity(0.0), isHumidityValid(false) {
        librarySensor.begin(); //< Initialize sensor library
    }

    /**
     * @brief Measures tempeature and humidity
     *
     */
    void measure() {
        measuredTempeature = librarySensor.readTemperature();
        measuredHumidity = librarySensor.readHumidity();

        isTempeatureValid = !isnan(measuredTempeature); //Check if measurement returned values
        isHumidityValid = !isnan(measuredHumidity);
    }

    /**
     * @brief Measured tempeature
     *
     * Unit is °C
     *
     * @return float tempeature
     */
    float tempeature() {
        return measuredTempeature;
    }

    /**
     * @brief Is tempeature value valid
     *
     * @return true Tempeature is valid
     * @return false Sensor couldn't measure tempeature
     */
    bool validTempeature() {
        return isTempeatureValid;
    }

    /**
     * @brief Measured humidity
     *
     * Unit is %
     *
     * @return float humidity
     */
    float humidity() {
        return measuredHumidity;
    }

    /**
     * @brief Is humidity value valid
     *
     * @return true Humidity is valid
     * @return false Sensor couldn't measure humidity
     */
    bool validHumidity() {
        return isHumidityValid;
    }
};

/**
 * @brief GP2Y1010AU0F pollution sensor
 *
 */
class PollutionSensor {

    float measuredPollution;
    bool isPollutionValid;

public:
    PollutionSensor() : measuredPollution(0.0), isPollutionValid(false) {
        pinMode(pollutionDigitalPIN, OUTPUT); //< Initialize pollution sensor digital (PWM LED) pin
    }

    /**
     * @brief Measures pollution
     *
     */
    void measure() {
        //Time constats for waiting between measurement phases are taken from datasheet

        float measuredPollVoltage = 0;

        digitalWrite(pollutionDigitalPIN, LOW); //Turn of LED, wait for specified time, measure, wait and turn off LED
        delayMicroseconds(280);
        measuredPollVoltage = analogRead(pollutionAnalogPIN);
        delayMicroseconds(40);
        digitalWrite(pollutionDigitalPIN, HIGH);
        delayMicroseconds(9800);

        float measuredPollVoltageToDigital = measuredPollVoltage * (5.0 / 1024.0); //< Get measured voltahe
        measuredPollution = (0.170 * measuredPollVoltageToDigital - 0.11) * 1000.0; //< Map voltage to pollution, equation is deduced from datasheet's graph

        if (measuredPollution > 0) {  //Check if measurement returned values
            isPollutionValid = true;
        } else {
            isPollutionValid = false;
        }
    }

    /**
     * @brief Measured pollution
     *
     * Unit is ug/m3
     *
     * @return float pollution
     */
    float pollution() {
        return measuredPollution;
    }

    /**
     * @brief Is pollution value valid
     *
     * @return true Pollution is valid
     * @return false Sensor couldn't measure pollution
     */
    bool validPollution() {
        return isPollutionValid;
    }
};

/**
 * @brief MH-Z19B CO2 sensor
 *
 */
class CO2Sensor {

    float measuredCO2;
    bool isCO2Valid;

    MHZ19 * mhz19uart;

public:
    CO2Sensor() : mhz19uart(new MHZ19(co2TxPIN, co2RxPIN)) {
        mhz19uart->begin(co2TxPIN, co2RxPIN); //< Initialize sensor library, use UART for communication
    }

    /**
     * @brief Measures CO2
     *
     */
    void measure() {
        measurement_t co2measured = mhz19uart->getMeasurement(); //< Measure and map value
        measuredCO2 = map(co2measured.co2_ppm, 0, 5000, 0, 2000);

        if (measuredCO2 > 200) { //< Check if value is above threshold
            isCO2Valid = true;
        } else {
            isCO2Valid = false;
        }
    }

    /**
     * @brief Measured CO2
     *
     * Unit is ppm
     *
     * @return float CO2
     */
    float co2() {
        return measuredCO2;
    }

    /**
     * @brief
     *
     * @return float
     */
    float validCO2() {
        return isCO2Valid;
    }
};

//!SECTION

/**
 * @brief SSD1306 128x64 Display
 *
 */
class Display {
private:
    U8GLIB_SSD1306_128X64 libraryDisplay; //< Display library object

    /**
     * @brief Draw single text message to screen
     *
     * @param message Message to display
     * @param top Distance from top of screen
     */
    void redrawMessage(const String & message, int top) {
        libraryDisplay.setPrintPos(64 - (libraryDisplay.getStrWidth(message) / 2), top);
        libraryDisplay.print(message);
    }

    /**
     * @brief Draw measured data from sensor in current state
     *
     */
    void redrawMesurement() {
        String stringName;
        String stringValue(currentValue);  //< Data value
        stringValue += " ";

        categoryNameAndUnit(stringName, stringValue, currentState); //< Get name of the measurand and unit of data

        libraryDisplay.setPrintPos(64 - (libraryDisplay.getStrWidth(stringName.c_str()) / 2), 10); //Draw sensor name
        libraryDisplay.print(stringName.c_str());
        libraryDisplay.setPrintPos(64 - (libraryDisplay.getStrWidth(stringValue.c_str()) / 2), 30); //Draw data value and unit
        libraryDisplay.print(stringValue.c_str());
    }

public:
    /**
     * @brief State representing sensor from which to take data
     *
     */
    enum State {
        tempeature = 0,
        humidity = 1,
        pollution = 2,
        co2 = 3
    };

    bool measuring; //< Currently measuring
    bool noData; //< Sensor from current state has invalid data
    enum State currentState; //< Sensor to display data from
    float currentValue; //< Data for sensor from current state

    Display() : libraryDisplay(U8G_I2C_OPT_NONE), measuring(false), noData(true), currentState(tempeature), currentValue(0.0) {
        libraryDisplay.setFont(u8g_font_unifont); //< Set default font for display
    }

    void draw() {
        //Draw sequence
        libraryDisplay.firstPage();
        do {
            if (measuring) {
                redrawMessage("measuring...", 10);
            } else if (noData) {
                redrawMessage("no data", 30);
            } else {
                redrawMesurement();
            }
        } while (libraryDisplay.nextPage());
    }

    /**
     * @brief Switch current state to next
     *
     */
    void raiseState() {
        if (currentState == 3) {
            currentState = 0;
        } else {
            currentState++;
        }
    }

    /**
     * @brief Get name and unit for each sensor
     *
     * Appends values to inputted parameter
     *
     * @param[inout] name Name of the measurand
     * @param[inout] unit Unit of the measurand
     * @param state Sensor to get name and unit from
     */
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

EthernetServer server(80);  //< HTTP server

/**
 * @brief Copy data from sensor to display objects and draw on display
 *
 */
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

/**
 * @brief Turn on LED based on measured values
 *
 */
void lightenLED() {
    digitalWrite(redLedPIN, LOW);
    digitalWrite(blueLedPIN, LOW);

    //If CO2 is > 1000 ppm or pollution is > 300 turn on RED light, else light BLUE light
    if (co2Sensor->co2() > 1000
        || pollutionSensor->pollution() > 300) {
        digitalWrite(redLedPIN, HIGH);
    } else {
        digitalWrite(blueLedPIN, HIGH);
    }
}

/**
 * @brief Send HTTP table with values to client
 *
 * @param client Client to send to
 * @param name Name of the measurand
 * @param unit Unit of the measurand
 * @param value Value of measurement
 * @param validData Is the measured data valid
 */
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

/**
 * @brief Send HTTP response to client
 *
 * @param client Client to send to
 */
void createServerResponse(EthernetClient & client) {
    //HTTP header
    client.println("HTTP/1.1 200 OK");
    client.println("Content-Type: text/html");
    client.println("Connection: close");
    client.println();

    //HTTP file
    client.println("<!DOCTYPE HTML>");
    client.println("<html>");

    client.println("<h1>Air Quality Measurement Station</h1>");

    client.println("<table>");

    client.println("<tr>");
    client.println("<th>Sensor</th>");
    client.println("<th>Value</th>");
    client.println("<th>Unit</th>");
    client.println("</tr>");

    //For each sensor get name of measurand, unit of data and data value and send it to client as HTTP table row
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

/**
 * @brief Measure values from all sensors
 *
 */
void measureAll() {
    Serial.println("Measurement:: Measuring");

    display->measuring = true;
    display->draw();

    tempeatureAndHumiditySensor->measure();
    pollutionSensor->measure();
    co2Sensor->measure();

    display->measuring = false;
    passToDisplay(); //< Display measured values

    lightenLED(); //< Light LED to correct color based on measured values
}
//!SECTION

//SECTION: Arduino methods
void setup() {
    Serial.begin(9600);

    //Initialize sensors objects
    tempeatureAndHumiditySensor = new TempeatureHumiditySensor();
    pollutionSensor = new PollutionSensor();
    co2Sensor = new CO2Sensor();

    //Initialize display object
    display = new Display();
    display->draw();

    //Initialize Ethernet and Server
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

    //Initialize button
    pinMode(buttonPIN, INPUT_PULLUP);

    //Initialize LED
    pinMode(redLedPIN, OUTPUT);
    pinMode(blueLedPIN, OUTPUT);

    measureAll(); //< Do first measure
}

unsigned long lastActionTime = 0; //< Time of last measurement, used for measurement timing 
unsigned long lastDebounceTime = 0; //< Time of last button press, used for button debounce and no-repeat

void loop() {

    if (millis() - lastActionTime > measureRate) {  //< If the time is correct, perform a new measurement on all sensors
      measureAll();
      lastActionTime = millis();
    }

    if (((millis() - lastDebounceTime) > 200)
        && (digitalRead(buttonPIN) == LOW && !display->measuring)) { //<If the button is not pressed during the debounce period, switch the screen on the display
        display->raiseState();
        passToDisplay();
        lastDebounceTime = millis();
    }

    // Server logic
    EthernetClient client = server.available();
    if (client && !display->measuring) { //< If new connection
        Serial.println("Server:: New connection");
        bool blankRequestLine = true; //< Blank request line indicates the end of HTTP header
        while (client.connected()) {
            if (client.available()) {
                //< Read the header until blankline with newline (end of header)
                char c = client.read();

                if (c == '\n' && blankRequestLine) {
                    createServerResponse(client); //< Send data to client
                    break;
                }
                if (c == '\n') {
                    blankRequestLine = true;
                } else if (c != '\r') {
                    blankRequestLine = false;
                }
            }
        }
        //Wait for client and stop the connection
        delay(1);
        client.stop();
        Serial.println("Server:: Ended connection");
    }
}
//!SECTION