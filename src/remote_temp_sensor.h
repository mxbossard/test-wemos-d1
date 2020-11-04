#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#include <Wire.h>
#include <Adafruit_BME280.h>


#if defined(ESP8266)
 #include <ESP8266WiFi.h>
 #include <ESP8266HTTPClient.h>
#elif defined(ESP32)
 #include <WiFi.h>
 #include <HTTPClient.h>
#endif

#include <privateCredentials.h>

#define D0 16
#define D1 5
#define D2 4
#define D3 0
#define D4 2
#define D5 14
#define D6 12
#define D7 13
#define D8 15

#define DHTTYPE DHT22

#define HTTP_PROTOCOL "http"
#define HTTP_HOST "192.168.0.16"
#define HTTP_PORT 3000
#define HTTP_ENDPOINT "/tao"

// Define the WiFi settings.
const char *ssid = WIFI_PRIVATE_SSID;
const char *password = WIFI_PRIVATE_PASSWORD;

const char *httpProtocol = HTTP_PROTOCOL;
const char *httpHost = HTTP_HOST;
const int httpPort = HTTP_PORT;
const char *httpEndpoint = HTTP_ENDPOINT;
const String httpUrl = String(httpProtocol) + "://" + String(httpHost) + ":" +  String(httpPort) + String(httpEndpoint);

// DHT sensor connected to D7
DHT_Unified dht(D7, DHTTYPE);

// BME280 sensor on I2C
Adafruit_BME280 bme; // use I2C interface
Adafruit_Sensor *bme_temp = bme.getTemperatureSensor();
Adafruit_Sensor *bme_pressure = bme.getPressureSensor();
Adafruit_Sensor *bme_humidity = bme.getHumiditySensor();

void WiFiOff() {
    WiFi.mode(WIFI_OFF);
    WiFi.forceSleepBegin();   // turn off ESP8266 RF
    delay(1);
}

void waitForWiFiConnection() {
    // Halt the code until connected to WiFi.
    int k = 0;
    while (WiFi.status() != WL_CONNECTED) {
        k++;
        delay(1000);
        Serial.print(".");
        if (k % 10 == 0) {
            Serial.print("Wi-Fi current status: ");
            Serial.println(WiFi.status());
        }
    }
}

void setup() {
    // Start Wi-Fi
    WiFi.mode(WIFI_STA);
    WiFi.setAutoReconnect(true);
    WiFi.begin(ssid, password);

    // Start PWM on D3 to start Charge Pump
    // Need to wait 2 sec before Charge Pump get decent level. 
    pinMode(D3, OUTPUT);
    analogWriteFreq(20000);
    analogWrite (D3, 512) ;

    Serial.begin(115200);

    // ##### BME280 management
    sensors_event_t temp_event, pressure_event, humidity_event;
    bme_temp->getEvent(&temp_event);
    bme_pressure->getEvent(&pressure_event);
    bme_humidity->getEvent(&humidity_event);



    // ##### DHT management
    // Initialize device.
    dht.begin();
    sensor_t sensor;
    uint32_t delayMS = sensor.min_delay / 1000;
    // Delay between measurements.
    delay(delayMS);
    // Get temperature event and print its value.
    sensors_event_t event;
    dht.temperature().getEvent(&event);
    if (isnan(event.temperature)) {
        Serial.println(F("Error reading temperature!"));
    }
    else {
        Serial.print(F("Temperature: "));
        Serial.print(event.temperature);
        Serial.println(F("°C"));
    }
    // Get humidity event and print its value.
    dht.humidity().getEvent(&event);
    if (isnan(event.relative_humidity)) {
        Serial.println(F("Error reading humidity!"));
    }
    else {
        Serial.print(F("Humidity: "));
        Serial.print(event.relative_humidity);
        Serial.println(F("%"));
    }
}
