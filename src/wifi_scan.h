/*
    This sketch demonstrates how to scan WiFi networks.
    The API is almost the same as with the WiFi Shield library,
    the most obvious difference being the different file you need to include:
*/
#include "ESP8266WiFi.h"
//#include "erase_config.h"

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_OFF);
  delay(1000);

  IPAddress local_ip = IPAddress(0,0,0,0);
  WiFi.persistent(false);
  WiFi.disconnect(true);
  WiFi.config(local_ip, local_ip, local_ip);    //Reset to use DHCP
  WiFi.hostname("esp_foo"); //This is the hostname that should be supplied to the DHCP server
  WiFi.setOutputPower(0);
  WiFi.setPhyMode(WIFI_PHY_MODE_11G);
  WiFi.setSleepMode(WIFI_NONE_SLEEP);

  uint8_t macAddr[6];
  WiFi.macAddress(macAddr);
  Serial.printf("mac address: %02x:%02x:%02x:%02x:%02x:%02x\n", macAddr[0], macAddr[1], macAddr[2], macAddr[3], macAddr[4], macAddr[5]);

  // Set WiFi to station mode and disconnect from an AP if it was previously connected
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);

  Serial.println("Setup done");
}

void loop() {
  WiFi.mode(WIFI_OFF);
  delay(1000);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);

  Serial.println("scan start");

  // WiFi.scanNetworks will return the number of networks found
  for (int k = 0; k < 1 ; k++) {
    Serial.print("scanning channel #");
    Serial.println(k);
    int n = WiFi.scanNetworks(false, true, k);
    Serial.println("scan done");
    if (n == 0) {
      Serial.println("no networks found");
    } else {
      Serial.print(n);
      Serial.println(" networks found");
      for (int i = 0; i < n; ++i) {
        // Print SSID and RSSI for each network found
        Serial.print(i + 1);
        Serial.print(": ");
        Serial.print(WiFi.SSID(i));
        Serial.print(" #");
        Serial.print(WiFi.channel(i));
        Serial.print(" (");
        Serial.print(WiFi.RSSI(i));
        Serial.print(")");
        Serial.println((WiFi.encryptionType(i) == ENC_TYPE_NONE) ? " " : "*");
        delay(10);
      }
    }
    Serial.println("");
    delay(100);
  }
  

  // Wait a bit before scanning again
  delay(1000);
}
