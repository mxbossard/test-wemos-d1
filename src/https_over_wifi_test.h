// Include required libraries to get data from your data panel connection page.
#include <ESP8266WiFi.h>
#include <WiFiClientSecure.h>
//#include <ESP8266WebServer.h>
#include <ESP8266HTTPClient.h>

// Define the WiFi settings.
const char *ssid = "Max-Box2";
const char *password = "ilovemaxbundy!";
// Define the hostname, the port number and the fingerprint.
const char *host = "mby.fr"; // "176.31.116.137"; //
const char fingerprint[] PROGMEM = "fingerprint";
const int httpsPort = 443;

const String webpage = "https://" + String(host) + "/";

void setup() {
    Serial.begin(115200);
    WiFi.mode(WIFI_OFF);
    delay(1000);

    Serial.print("Attempting to connect to Wi-Fi network named: ");
    Serial.println(ssid);                   // print the network name (SSID);

    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    // Halt the code until connected to WiFi.
    while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
        Serial.print(".");
    }
    Serial.println("");

    // print the SSID of the network you're attached to:
    Serial.print("SSID: ");
    Serial.println(WiFi.SSID());

    // print your WiFi shield's IP address:
    IPAddress ip = WiFi.localIP();
    Serial.print("IP Address: ");
    Serial.println(ip);
}

void loop() {

    // Create a WiFiClientSecure object.
    WiFiClientSecure client;
    // Set the fingerprint to connect the server.
    //client.setFingerprint(fingerprint);

    client.setInsecure();

    Serial.print("Attempting to connect to Host host: ");
    Serial.print(host);
    Serial.print(" on port: ");
    Serial.println(httpsPort);    
    // If the host is not responding,return.
    if(!client.connect(host, httpsPort)){
        Serial.println("Connection Failed!");
        delay(2000);
        return;
    }

    // Send a GET request to a web page hosted by the server.

    Serial.println("GET " + webpage + " HTTP/1.1");
    Serial.println("Host: " + String(host)); 
    Serial.println("Connection: close");
    Serial.println();

    client.println("GET " + webpage + " HTTP/1.1");
    client.println("Host: " + String(host)); 
    client.println("Connection: close");
    client.println();

    while (client.connected()) {
      String line = client.readStringUntil('\n');
      if (line == "\r") {
        Serial.println("headers received");
        break;
      }
    }

    // Detect whether client is responding properly or not.
    unsigned long timeout = millis();
    while (client.available() == 0) {
        if (millis() - timeout > 5000) {
            Serial.println(">>> Client Timeout !");
            client.stop();
            delay(10000);
            return;
        }
    }
    // if there are incoming bytes available
    // from the server, read them and print them:
    while (client.available()) {
      char c = client.read();
      Serial.write(c);
    }

    client.stop();
    delay(10000);
}