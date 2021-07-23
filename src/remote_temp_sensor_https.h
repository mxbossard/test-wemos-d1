#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#include <Wire.h>
#include <Adafruit_BME280.h>
#include <CayenneLPP.h>
#include <Hash.h>

#if defined(ESP8266)
 #include <ESP8266WiFi.h>
 #include <WiFiClientSecure.h>
 #include <ESP8266HTTPClient.h>
 //#include <WiFiClientSecureBearSSL.h>
#elif defined(ESP32)
 #include <WiFi.h>
 #include <HTTPClient.h>
#endif

#include "certs.h"
#include "privateCredentials.h"

#define D0 16
#define D1 5
#define D2 4
#define D3 0
#define D4 2
#define D5 14
#define D6 12
#define D7 13
#define D8 15

//#define DEBUG

#define DHTTYPE DHT22

#define HTTP_PROTOCOL "https"
#define HTTP_HOST "iot.mby.fr"
#define HTTP_PORT 443
#define HTTP_ENDPOINT "/api/v1/datastore/lpp"
#define HTTPS_SHA1_FINGERPRINT "C6:E6:92:C7:17:45:6A:54:FA:44:57:B3:49:C5:FB:A8:72:CD:64:67"
#define HTTPS_SHA256_FINGERPRINT "86CD250983B4B4B2956EC19FB9056E9280FFD47BFC15D69555C6B68C010BCA4E"
#define HTTP_MAXIMUM_POST_ATTEMPT 3

#define PROBE_MINIMUM_SIGNIFICANT_VALUES 3
#define PROBE_MAXIMUM_PROBING_ITERATION 5
#define ADC_CORRECTION_RATIO ( 1 )

#define PROBE_SLEEP_DELAY_IN_SEC 300

#ifdef DEBUG
  #define PROBE_SLEEP_DELAY_IN_SEC 5
#endif

#define LPP_BOARD_CHANNEL 0
#define LPP_BME280_CHANNEL 1
#define LPP_DHT22_CHANNEL 2
#define LPP_TIME_CHANNEL 42
#define LPP_BOARD_PROBE_COUNT_CHANNEL (100 + 0)
#define LPP_BME280_PROBE_COUNT_CHANNEL (100 + 1)
#define LPP_DHT22_PROBE_COUNT_CHANNEL (100 + 2)
#define LPP_BOARD_ERROR_COUNT_CHANNEL (200 + 0)
#define LPP_BME280_ERROR_COUNT_CHANNEL (200 + 1)
#define LPP_DHT22_ERROR_COUNT_CHANNEL (200 + 2)

// Define the WiFi settings.
const char *ssid = WIFI_PRIVATE_SSID;
const char *password = WIFI_PRIVATE_PASSWORD;

const char *httpProtocol = HTTP_PROTOCOL;
const char *httpHost = HTTP_HOST;
const int httpPort = HTTP_PORT;
const char *httpEndpoint = HTTP_ENDPOINT;
const char *httpsFingerprint = HTTPS_SHA1_FINGERPRINT;

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
    Serial.println("Wi-Fi disabled.");
}

void waitForWiFiConnection() {
    Serial.println("Waiting for connection to Wi-Fi ...");
    // Halt the code until connected to WiFi.
    while (WiFi.status() != WL_CONNECTED) {
        delay(10);
    }
    Serial.println("Connexted to Wi-Fi.");
}

/**
 * Return [temperature, humidty, pressure]
 */ 
float * probeBme280() {
    //Serial.println(F("Probing BME280 ..."));
    bool error = false;
    float temp = 0;
    float humidity = 0;
    float pressure = 0;

    if (bme.begin(0x76, & Wire)) {
        sensors_event_t temp_event, pressure_event, humidity_event;
        bme_temp->getEvent(&temp_event);
        bme_pressure->getEvent(&pressure_event);
        bme_humidity->getEvent(&humidity_event);

        //Serial.println(F("Probing BME280 done."));

        /*
        Serial.print(F("Temperature = "));
        Serial.print(temp_event.temperature);
        Serial.println(" *C");

        Serial.print(F("Humidity = "));
        Serial.print(humidity_event.relative_humidity);
        Serial.println(" %");

        Serial.print(F("Pressure = "));
        Serial.print(pressure_event.pressure);
        Serial.println(" hPa");
        */

        temp = temp_event.temperature;
        humidity = humidity_event.relative_humidity;
        pressure = pressure_event.pressure;
    } else {
        Serial.println(F("Could not find a valid BME280 sensor, check wiring!"));
        error = true;
    }

    static float array[4];
    array[0] = temp;
    array[1] = humidity;
    array[2] = pressure;
    array[3] = error;
    return array;
}

/**
 * Return [temperature, humidty]
 */ 
float * probeDht22() {
    //Serial.println(F("Probing DHT22 ..."));
    dht.begin();

    sensors_event_t temp_event, humidity_event;
    dht.temperature().getEvent(&temp_event);
    dht.humidity().getEvent(&humidity_event);

    float temp = temp_event.temperature;
    float humidity = humidity_event.relative_humidity;
    //Serial.println(F("Probing DHT22 done."));

    /*
    Serial.print(F("Temperature: "));
    Serial.print(temp_event.temperature);
    Serial.println(F("°C"));

    Serial.print(F("Humidity: "));
    Serial.print(humidity_event.relative_humidity);
    Serial.println(F("%"));
    */

    bool error = isnan(temp) || isnan(humidity);

    static float array[3];
    array[0] = temp;
    array[1] = humidity;
    array[2] = error;
    return array;
}

float readADC() {
    //Serial.println(F("Reading ADC ..."));
    float ad = 0;
    //float resolution = 3.13 * (2.187+2.200) / 2.187 / 1023; //calibrate based on your voltage divider AND Vref!
    float resolution = 3.2 * 2 * ADC_CORRECTION_RATIO / 1023;
    int adcr = analogRead(0);
    //Serial.println("Read ADC value: " + String(adcr));
    //Serial.println("Resolution: " + String(resolution, 10) + "mV");
    ad = adcr * resolution;
    //Serial.println(F("Reading ADC done."));

    return ad;
}

int sortFloatAsc(const void *cmp1, const void *cmp2) {
    // Need to cast the void * to float *
    float a = *((float *)cmp1);
    float b = *((float *)cmp2);

    // NAN values get pushed at the end of the array
    if (isnan(a)) return 1;
    if (isnan(b)) return -1;

    // The comparison
    //return a > b ? 1 : (a < b ? -1 : 0);
    // A simpler, probably faster way:
    return a - b;
}

/**
 * Return the median of a float array.
 * Exclude NAN values.
 * return a NAN value if less than PROBE_MINIMUM_SIGNIFICANT_VALUES non NAN values in the array.
 */
float medianOfArray(float array[]) {
    //Serial.println("medianOfArray with array[0]: " + String(array[0]));
    int arrayLength = sizeof(array) - 1;
    int arraySize = arrayLength * sizeof(array[0]);
    //Serial.println("arrayLength: " + String(arrayLength) + " ; arraySize: " + String(arraySize));
    float buffer[arrayLength];
    memcpy(buffer, array, arraySize);
    //Serial.println("buffer[0] before sort: " + String(buffer[0]));
    //Serial.println("buffer[1] before sort: " + String(buffer[1]));
    //Serial.println("buffer[2] before sort: " + String(buffer[2]));
    
    // qsort - last parameter is a function pointer to the sort function
    qsort(buffer, arrayLength, sizeof(array[0]), sortFloatAsc);
    //Serial.println("buffer[0] after sort: " + String(buffer[0]));
    //Serial.println("buffer[1] after sort: " + String(buffer[1]));
    //Serial.println("buffer[2] after sort: " + String(buffer[2]));
    
    // Count values in array (not NAN values)
    byte valuesCount = 0;
    for (int k=0; k < arrayLength; k++) {
        if (!isnan(array[k])) valuesCount++;
    }

    // If too few significant values, return NAN.
    if (valuesCount < PROBE_MINIMUM_SIGNIFICANT_VALUES) {
        return NAN;
    }

    int medianIndex = (valuesCount + 1) / 2;
    float medianValue = buffer[medianIndex];
    Serial.println("Median value: " + String(medianValue) + " from " + String(valuesCount) + " values.");
    return medianValue;
}

void array_to_string(byte array[], unsigned int len, char buffer[])
{
    for (unsigned int i = 0; i < len; i++)
    {
        byte nib1 = (array[i] >> 4) & 0x0F;
        byte nib2 = (array[i] >> 0) & 0x0F;
        buffer[i*2+0] = nib1  < 0xA ? '0' + nib1  : 'A' + nib1  - 0xA;
        buffer[i*2+1] = nib2  < 0xA ? '0' + nib2  : 'A' + nib2  - 0xA;
    }
    buffer[len*2] = '\0';
}

void doWork() {
    // Start Wi-Fi
    WiFi.mode(WIFI_STA);
    WiFi.setAutoReconnect(false);
    if (strcmp("", password) == 0) {
        // No WiFI password
        Serial.println("Connecting WiFi without password ...");
        WiFi.begin(ssid);
    } else {
        Serial.println("Connecting WiFi with a password ...");
        WiFi.begin(ssid, password);
    }

    // Power on sensors through D8
    pinMode(D8, OUTPUT);
    digitalWrite(D8, HIGH);

    // Start PWM on D3 to start Charge Pump
    // Need to wait 2 sec before Charge Pump get decent level. 
    pinMode(D3, OUTPUT);
    analogWriteFreq(20000);
    analogWrite (D3, 512) ;

    Serial.begin(115200);
    Serial.println("");
   
    uint32_t espUid = ESP.getChipId();
    String espMacAddress = WiFi.macAddress();
    String uid = String(espUid) + "_" + espMacAddress;
    String sensorLongId = sha1(uid);
    String sensorShortId = sensorLongId.substring(0, 7);
    Serial.println("sensorShortId: " + sensorShortId + " ; UID: " + uid + " ; sensorLongId: " + sensorLongId);


    // ---------------- HTTPS config block start -----------------

    // Set time via NTP, as required for x.509 validation
    configTime(3 * 3600, 0, "pool.ntp.org", "time.nist.gov");

    Serial.print("Waiting for NTP time sync: ");
    time_t now = time(nullptr);
    while (now < 8 * 3600 * 2) {
        delay(500);
        Serial.print(".");
        now = time(nullptr);
    }
    Serial.println("");
    struct tm timeinfo;
    gmtime_r(&now, &timeinfo);
    Serial.print("Current time: ");
    Serial.println(asctime(&timeinfo));

    // Use WiFiClientSecure class to create TLS connection
    WiFiClientSecure wifiClient;
    wifiClient.stop();
    X509List cert(cert_mby_fr);

    waitForWiFiConnection();

    bool connectionKo = true;
    int connectionAttempt = 0;
    int maxConnectionAttempt = 3;
    while (connectionKo && connectionAttempt < maxConnectionAttempt) {
        connectionAttempt ++;
        
        Serial.print("Connecting to ");
        Serial.print(httpHost);
        Serial.print(" attempt #");
        Serial.println(connectionAttempt);
        
        //Serial.printf("Using certificate: %s\n", cert_mby_fr);
        wifiClient.setTrustAnchors(&cert);
        if (!wifiClient.connect(httpHost, httpPort)) {
            Serial.println("Connection failed");
            delay(1000);
        } else {
            Serial.println("Connection succeeded");
            connectionKo = false;
        }
    }

    if (connectionAttempt == maxConnectionAttempt) {
        Serial.print("Unable to connection to ");
        Serial.println(httpHost);
        return;
    }

    // ---------------- HTTPS config block end -----------------

    HTTPClient httpClient;

    /*
    sensor_t sensor;
    Adafruit_Sensor * sensors[5] = {bme_temp, bme_humidity, bme_pressure, dht_temp, dht_humidity};
    //for(Adafruit_Sensor as : sensors) {
    for (size_t i = 0; i < sizeof(sensors); i++) {
        sensors[i]->getSensor(&sensor);
        Serial.println("Checking min_delay of sensor " + String(sensor.sensor_id));
        probingDelay = max(probingDelay, sensor.min_delay / 1000);
    }
    
    */
    // Wait 2 sec to be sure Charge Pump is stabilized
    delay(2000);

    Wire.begin(D2, D1);

    int probingDelay = 2000;
    byte errorCount = 0;
    byte probeCount = PROBE_MINIMUM_SIGNIFICANT_VALUES;
    byte maxProbeCount = PROBE_MAXIMUM_PROBING_ITERATION;

    float batVoltageValues[maxProbeCount];
    float bme280tempValues[maxProbeCount];
    float bme280HumidityValues[maxProbeCount];
    float bme280PressureValues[maxProbeCount];
    float dht22tempValues[maxProbeCount];
    float dht22HumidityValues[maxProbeCount];

    // Fill arrays with NAN
    for (int k=0; k < maxProbeCount; k++) {
        batVoltageValues[k] = NAN;
        bme280tempValues[k] = NAN;
        bme280HumidityValues[k] = NAN;
        bme280PressureValues[k] = NAN;
        dht22tempValues[k] = NAN;
        dht22HumidityValues[k] = NAN;   
    }

    Serial.println("");

    for (int k=0; k < probeCount; k++) {
        Serial.println("Probing #" + String(k + 1) + " of " + String(probeCount) + " ...");
        
        float batVoltage = readADC();
        batVoltageValues[k] = batVoltage;
        //Serial.println("Battery voltage: " + String(batVoltage) + " V");

        float * bme280Datas = probeBme280();
        bool bme280Error = bme280Datas[3];
        if (!bme280Error) {
            bme280tempValues[k] = bme280Datas[0];
            bme280HumidityValues[k] = bme280Datas[1];
            bme280PressureValues[k] = bme280Datas[2];
        }
        //Serial.println("BME 280 datas: " + String(bme280Datas[0]) + " ; " + String(bme280Datas[1]) + " ; " + String(bme280Datas[2]));

        float * dht22Datas = probeDht22();
        bool dht22Error = dht22Datas[2];
        if (!dht22Error) {
            dht22tempValues[k] = dht22Datas[0];
            dht22HumidityValues[k] = dht22Datas[1];
        }
        //Serial.println("DHT 22 datas: " + String(dht22Datas[0]) + " ; " + String(dht22Datas[1]));
        
        if (probeCount < maxProbeCount && (bme280Error || dht22Error)) {
            probeCount ++;
            errorCount ++;
        }

        // Probe delay only befor probing
        if (k < probeCount - 1) {
            //Serial.println("Waiting for probingDelay: " + String(probingDelay) + " ...");
            delay(probingDelay);
        }
    }

    if (errorCount > 0) {
        Serial.println("/!\\ Got some errors !");
    }

    float batVoltageValue = medianOfArray(batVoltageValues);
    float bme280tempValue = medianOfArray(bme280tempValues);
    float bme280HumidityValue = medianOfArray(bme280HumidityValues);
    float bme280PressureValue = medianOfArray(bme280PressureValues);
    float dht22tempValue = medianOfArray(dht22tempValues);
    float dht22HumidityValue = medianOfArray(dht22HumidityValues);

    CayenneLPP lpp(128);
    lpp.reset();
    if (!isnan(batVoltageValue))
        lpp.addVoltage(LPP_BOARD_CHANNEL, batVoltageValue);
    if (!isnan(bme280tempValue))
        lpp.addTemperature(LPP_BME280_CHANNEL, bme280tempValue);
    if (!isnan(bme280HumidityValue))
        lpp.addRelativeHumidity(LPP_BME280_CHANNEL, bme280HumidityValue);
    if (!isnan(bme280PressureValue))
        lpp.addBarometricPressure(LPP_BME280_CHANNEL, bme280PressureValue);
    if (!isnan(dht22tempValue))
        lpp.addTemperature(LPP_DHT22_CHANNEL, dht22tempValue);
    if (!isnan(dht22HumidityValue))
        lpp.addRelativeHumidity(LPP_DHT22_CHANNEL, dht22HumidityValue);
    
    // Error count
    lpp.addDigitalInput(LPP_BOARD_ERROR_COUNT_CHANNEL, errorCount);
    lpp.addDigitalInput(LPP_BME280_ERROR_COUNT_CHANNEL, errorCount);
    lpp.addDigitalInput(LPP_DHT22_ERROR_COUNT_CHANNEL, errorCount);

    // Probe count
    lpp.addDigitalInput(LPP_BOARD_PROBE_COUNT_CHANNEL, probeCount);
    lpp.addDigitalInput(LPP_BME280_PROBE_COUNT_CHANNEL, probeCount);
    lpp.addDigitalInput(LPP_DHT22_PROBE_COUNT_CHANNEL, probeCount);

    // Probing time
    //time(&now);
    uint32_t probingTime = (uint32_t) now;
    Serial.print("Unix time: ");
    Serial.println(probingTime);
    //lpp.addUnixTime(LPP_TIME_CHANNEL, probingTime);
    lpp.addUnixTime(LPP_TIME_CHANNEL, probingTime);
    
#ifdef DEBUG
    DynamicJsonDocument jsonBuffer(2048);
    JsonArray root = jsonBuffer.to<JsonArray>();
    lpp.decode(lpp.getBuffer(), lpp.getSize(), root);
    //serializeJsonPretty(root, Serial);
    serializeJson(root, Serial);
    Serial.println("");
#endif

    /*
    String batteryVoltage = String(batVoltageValue, 3);
    String bme280Temp = String(bme280tempValue, 2);
    String bme280Humidity = String(bme280HumidityValue, 1);
    String bme280Pressure = String(bme280PressureValue, 1);
    String dht22Temp = String(dht22tempValue, 2);
    String dht22Humidity = String(dht22HumidityValue, 1);
    
    Serial.println("Battery voltage: " + batteryVoltage + " V");
    Serial.println("BME 280 temperature: " + bme280Temp + " °C");
    Serial.println("BME 280 humidity: " + bme280Humidity + " %");
    Serial.println("BME 280 pressure: " + bme280Pressure + " hPa");
    Serial.println("DHT 22 temperature: " + dht22Temp + " °C");
    Serial.println("DHT 22 humidity: " + dht22Humidity + " %");

    Serial.println("");

    String payload = "{\"data\":{\"uid\": " + String(espUid) + ", \"mac\": \"" + macAddress + "\", \"bme280\": {\"temperature\": " + bme280Temp + ", \"humidity\": " + bme280Humidity + ", \"pressure\": " + bme280Pressure + "}, \"dht22\": {\"temperature\": " + dht22Temp + ", \"humidity\": " + dht22Humidity + "}, \"battery\":" + batteryVoltage + ", \"errors\":" + errorCount + "}}";
    Serial.println("Payload JSON: " + payload);
    */

    String postUrl = httpUrl + "/" + sensorShortId + "_v0";
    Serial.print("Posting data to URL: ");
    Serial.println(postUrl);

    char str[128] = "";
    array_to_string(lpp.getBuffer(), lpp.getSize(), str);
    Serial.print("LPP: ");
    Serial.println(str);

    bool dataNotPosted = true;
    int postingAttempt = 0;

    while (dataNotPosted and postingAttempt < HTTP_MAXIMUM_POST_ATTEMPT) {
        postingAttempt ++;
        httpClient.begin(wifiClient, postUrl);
        httpClient.setFollowRedirects(HTTPC_FORCE_FOLLOW_REDIRECTS);
        httpClient.setRedirectLimit(10);
        //httpClient.addHeader("Content-Type", "application/json");
        httpClient.addHeader("Content-Type", "application/octet-stream");

        Serial.println("Data posting attempt #" + String(postingAttempt) + " ..." );
        int httpCode = httpClient.POST(lpp.getBuffer(), lpp.getSize());
        // End necessary to repost request.
        httpClient.end();

        if (httpCode == 200) {
            dataNotPosted = false;
            Serial.println("Data successfully posted.");
        } else {
            Serial.println("Data not posted ! HTTP status was: " + String(httpCode));
        }
    }

    if (postingAttempt == HTTP_MAXIMUM_POST_ATTEMPT) {
        Serial.print("Unable to post data !");
        Serial.println(httpHost);
        return;
    }

    wifiClient.stop();

    WiFiOff();

    Serial.println("Work done.");
}

void setup() {

}

void loop() {
    Serial.println("Launching doWork() ...");
    doWork();

#ifndef DEBUG
    ESP.deepSleep(1000000 * PROBE_SLEEP_DELAY_IN_SEC);
#endif

    Serial.println("Not sleeping.");

    Serial.println("Will wait for " + String(PROBE_SLEEP_DELAY_IN_SEC) + " seconds ...");
    delay(1000 * PROBE_SLEEP_DELAY_IN_SEC);
}

