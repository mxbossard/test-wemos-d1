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
Adafruit_Sensor *dht_temp = dht.temperature();
Adafruit_Sensor *dht_humidity = dht.humidity();

// BME280 sensor on I2C
Adafruit_BME280 bme; // use I2C interface
Adafruit_Sensor *bme_temp = bme.getTemperatureSensor();
Adafruit_Sensor *bme_humidity = bme.getHumiditySensor();
Adafruit_Sensor *bme_pressure = bme.getPressureSensor();

HTTPClient client;

void WiFiOff() {
    WiFi.mode(WIFI_OFF);
    WiFi.forceSleepBegin();   // turn off ESP8266 RF
    delay(1);
}

void waitForWiFiConnection() {
    // Halt the code until connected to WiFi.
    while (WiFi.status() != WL_CONNECTED) {
        delay(10);
    }
}

/**
 * Return [temperature, humidty, pressure]
 */ 
float * probeBme280() {
    static float array[3];
    sensors_event_t temp_event, pressure_event, humidity_event;
    bme_temp->getEvent(&temp_event);
    bme_pressure->getEvent(&pressure_event);
    bme_humidity->getEvent(&humidity_event);

    array[0] = temp_event.temperature;
    array[1] = humidity_event.relative_humidity;
    array[2] = pressure_event.pressure;
    return array;
}

/**
 * Return [temperature, humidty]
 */ 
float * probeDht22() {
    static float array[2];
    sensors_event_t temp_event, humidity_event;
    dht_temp->getEvent(&temp_event);
    dht_humidity->getEvent(&humidity_event);

    array[0] = temp_event.temperature;
    array[1] = humidity_event.relative_humidity;
    return array;
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

    sensor_t sensor;

    int probingDelay = 1000;
    Adafruit_Sensor * sensors = [bme_temp, bme_humidity, bme_pressure, dht_temp, dht_humidity];
    for(Adafruit_Sensor &as : sensors) {
        as->getSensor(&sensor);
        Serial.println("Checking min_delay of sensor " + as->sensor_id);
        probingDelay = max(probingDelay, sensor.min_delay / 1000);
    }

    // Wait 2 sec to be sure Charge Pump is stabilized
    delay(2000);

    byte probeCount = 3;
    for (int k=0; k < probeCount; k++) {
        Serial.println("Probing #" + k + " of " + probeCount " attempts...");
        float * bme280DDatas = probeBme280();
        float * dht22Datas = probeDht22();

        Serial.println("BME 280 datas: " + bme280DDatas[0] + " ; " + bme280DDatas[1] + " ; " + bme280DDatas[2]);
        Serial.println("DHT 22 datas: " + dht22Datas[0] + " ; " + dht22Datas[1]);

        delay(probingDelay);
    }

    // Initialize device.
    dht.begin();
    sensor_t sensor;
    uint32_t delayMS = sensor.min_delay / 1000;

    // Delay between measurements.
    delay(delayMS);

    // Get temperature event and print its value.
    
}

void loop() {}