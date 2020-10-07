#include <Arduino.h>
#include <U8g2lib.h>

#include <ArduinoJson.h>
#include <Wire.h>

U8G2_SSD1306_64X48_ER_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE, 5, 4);   // EastRising 0.66" OLED breakout board, Uno: A4=SDA, A5=SCL, 5V powered

const char jsonPayload[] = "{\"pages\": [{\"text\": \"foo bar\"}, {\"text\": \"Lorem ipsum\"}, {\"text\": \"Lorem ipsum dolor\"}, {\"text\": \"Lorem ipsum dolor sit amet\"}, {\"text\": \"Lorem ipsum dolor sit amet, consectetur adipiscing elit.\"}, {\"text\": \"foo baz\"}, {\"text\": \"ploof\"}]}";

const int SCREEN_WIDTH = 64;
const int SCREEN_HEIGHT = 48;

const int FONT_WIDTH = 5;
const int FONT_HEIGHT = 8;


// Include required libraries to get data from your data panel connection page.
#include <ESP8266WiFi.h>
//#include <ArduinoHttpClient.h>

//#include <ESP8266WebServer.h>
//#include <ESP8266HTTPClient.h>

#define HTTP 1
#define HTTPS 2

#define HTTP_PROTOCOL HTTP
#define HTTP_HOST "192.168.0.16"
#define HTTP_PORT 3000
#define HTTP_ENDPOINT "/tao"

#if HTTP_PROTOCOL == HTTP
#include <WiFiClient.h>
const char *httpProtocol = "http";
#elif HTTP_PROTOCOL == HTTPS
#include <WiFiClientSecure.h>
const char *httpProtocol = "https";
#else
    #error Not supported http protocol !
#endif


// Define the WiFi settings.
#include <privateCredentials.h>
const char *ssid = WIFI_PRIVATE_SSID;
const char *password = WIFI_PRIVATE_PASSWORD;

const char *httpHost = HTTP_HOST;
const int httpPort = HTTP_PORT;
const char *httpEndpoint = HTTP_ENDPOINT;

const String webpage = String(httpProtocol) + "://" + String(httpHost) + String(httpEndpoint);

DynamicJsonDocument doc(4096);

void setupU8g2(void) {
    u8g2.begin();
    //u8g2.setFont(u8g2_font_micro_tr);
    //u8g2.setFont(u8g2_font_4x6_tf);
    //u8g2.setFont(u8g2_font_5x7_tf);
    u8g2.setFont(u8g2_font_5x8_tf);
    //u8g2.setFont(u8g2_font_6x10_tf);
    //u8g2.setFont(u8g2_font_cu12_hf);
    u8g2.setFontRefHeightExtendedText();
    u8g2.setDrawColor(1);
    u8g2.setFontPosTop();
    u8g2.setFontDirection(0);
    //u8g2.enableUTF8Print();

    u8g2.clear();
}

void setupWiFiBlocking(void) {
    //WiFi.mode(WIFI_OFF);
    //delay(1000);

    WiFi.mode(WIFI_STA);
    WiFi.setAutoReconnect(true);
    WiFi.begin(ssid, password);

    Serial.print(F("Attempting to connect to Wi-Fi network named: "));
    Serial.println(ssid);

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

WiFiClient buildWiFiClient(const char* protocol) {
    if (strcmp(protocol, "http") == 0) {
        WiFiClient client;
        return client;
    } else if (strcmp(protocol, "https") == 0) {
        WiFiClientSecure client;
        client.setInsecure();
        return client;
    }
    //FIXME throw Error;
}

bool connectWiFiClient(WiFiClient* client, const char* protocol, const char* host, int port) {
    Serial.print("Attempting to connect to Host host: ");
    Serial.print(host);
    Serial.print(" on port: ");
    Serial.print(port);
    Serial.print(" with protocol: ");
    Serial.print(protocol);
    Serial.println();

    if(!client->connect(host, port)) {
        Serial.println("Connection Failed!");
        return false;
    }
    
    Serial.println("Connected.");

    return true;
}

void sendHttpGet(WiFiClient* client, const char* hostHeader, const char* endpoint) {
    Serial.println("GET " + String(endpoint) + " HTTP/1.1");
    Serial.println("Host: " + String(hostHeader)); 
    Serial.println("Connection: close");
    Serial.println();

    client->println("GET " + String(endpoint) + " HTTP/1.1");
    client->println("Host: " + String(hostHeader)); 
    client->println("Connection: close");
    client->println();

    while (client->connected()) {
        String line = client->readStringUntil('\n');
        if (line == "\r") {
            Serial.println("headers received");
            break;
        }
    }

    // Detect whether client is responding properly or not.
    unsigned long timeout = millis();
    while (client->available() == 0) {
        if (millis() - timeout > 5000) {
            Serial.println(">>> Client Timeout !");
            client->stop();
            delay(10000);
            return;
        }
        delay(1);
    }
}

void displayPage(JsonObject page) {
    if (!page) return;
    const char* text = page["text"];

    Serial.print(F("Will display: "));
    Serial.println(text);

    u8g2.clear();

    // Code snippet : wrap a string to display

    // Allocate 2 bytes of memory to currentChar.
    char* currentChar = (char*) malloc(sizeof(char) * 2);
    char* nextChar = (char*) malloc(sizeof(char) * 2);
    // Init properly currentChar with an ending char array.
    strncpy(currentChar, "a", sizeof(char) * 2);
    strncpy(nextChar, "a", sizeof(char) * 2);

    uint16_t i, x, y;
    // u8x8 does not wrap lines.
    x = 0; y = 0;
    for (i = 0 ; i < strlen(text) && (y + FONT_HEIGHT) <= SCREEN_HEIGHT ; i ++) {
        strncpy(currentChar, &text[i], sizeof(char) * 1);

        int charWidth = u8g2.getStrWidth(currentChar);
        if (strcmp(currentChar, " ") == 0) {
            // Special fix reduced width for spaces
            charWidth = charWidth / 2;
        }
        
        if (x + charWidth > SCREEN_WIDTH) {
            // Reaching end of line
            if ((i + 1) < strlen(text)) {
                strncpy(nextChar, &text[i + 1], sizeof(char) * 1);
                if (strcmp(nextChar, " ") != 0) {
                    // if following char not a space print a dash
                    u8g2.print("-");
                }
            }
            x = 0;
            y += FONT_HEIGHT + 2;
        }

        if (x == 0 && strcmp(currentChar, " ") == 0) {
            // Remove spaces if first line chars
            continue;
        }

        u8g2.setCursor(x, y);
        u8g2.print(text[i]);
        /*
        Serial.print("x: ");
        Serial.print(x);
        Serial.print(" ; y: ");
        Serial.print(y);
        Serial.print(" ; width: ");
        Serial.print(charWidth);
        Serial.print(" ; char: ");
        Serial.println(currentChar);
        */
        
        x += charWidth + 1;
    }

    u8g2.sendBuffer();
}

void displayPages(DynamicJsonDocument doc) {
    JsonArray pages = doc["pages"];
    if (!pages) return;

    //Serial.println("Will display some pages ...");

    // Walk the JsonArray efficiently
    for(JsonObject page: pages) {
        displayPage(page);
        delay(3000);
    }
}

void quickDisplayText(String text) {
    String jsonPayload = "{\"pages\": [{\"text\": \"" + text + "\"}]}";

    DeserializationError err = deserializeJson(doc, jsonPayload);
    if(err) {
        Serial.print(F("deserializeJson() failed with code "));
        Serial.println(err.c_str());
    }

    displayPages(doc);
}

void setup(void) {
    Serial.begin(115200);
    Serial.println();

    setupU8g2();

    String message = "Setting up WiFi on " + String(ssid) + " ...";
    quickDisplayText(message);
    setupWiFiBlocking();

    Serial.println("");

    // print the SSID of the network you're attached to:
    Serial.print("SSID: ");
    Serial.println(WiFi.SSID());

    // print your WiFi shield's IP address:
    IPAddress ip = WiFi.localIP();
    Serial.print("IP Address: ");
    Serial.println(ip);
}

void loop(void) {
    // picture loop

     quickDisplayText(F("HTTP query ..."));

    // Create a WiFiClient object for http.
    WiFiClient client = buildWiFiClient(httpProtocol);

    bool connected = connectWiFiClient(&client, httpProtocol, httpHost, httpPort);

    if (connected) {
        // Send a GET request to a web page hosted by the server.

        sendHttpGet(&client, httpHost, httpEndpoint);
        
        DeserializationError err = deserializeJson(doc, client);

        if(err) {
            Serial.print(F("deserializeJson() failed with code "));
            Serial.println(err.c_str());
        }
    }

    displayPages(doc);

    client.stop();

    //const char* text = doc["pages"][0]["text"];
    

    // deley between each page
    delay(3000);

}
