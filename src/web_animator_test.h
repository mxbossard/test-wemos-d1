
#if defined(ESP8266)
 #include <ESP8266WiFi.h>
 #include <ESP8266HTTPClient.h>
#elif defined(ESP32)
 #include <WiFi.h>
 #include <HTTPClient.h>
#endif

#include <WebAnimator.h>

#define D0 16
#define D1 5
#define D2 4
#define D3 0
#define D4 2
#define D5 14
#define D6 12
#define D7 13
#define D8 15

#define SDA_PIN 4
#define SCL_PIN 5

U8G2_SSD1306_64X48_ER_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE, SCL_PIN, SDA_PIN);   // EastRising 0.66" OLED breakout board, Uno: A4=SDA, A5=SCL, 5V powered
//U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ DISPLAY_RESET_PIN, /* clock=*/ SCL_PIN, /* data=*/ SDA_PIN);

HTTPClient client;
WebAnimator webAnimator(u8g2);

const int HTTP_TIMEOUT = 10000;

#define HTTP 1
#define HTTPS 2

#define HTTP_PROTOCOL HTTP
#define HTTP_HOST "192.168.0.16"
#define HTTP_PORT 3000
#define HTTP_ENDPOINT "/tao"


// Define the WiFi settings.
const char *ssid = WIFI_PRIVATE_SSID;
const char *password = WIFI_PRIVATE_PASSWORD;

const char *httpHost = HTTP_HOST;
const int httpPort = HTTP_PORT;
const char *httpEndpoint = HTTP_ENDPOINT;

const String webpage = "http://" + String(httpHost) + ":" +  String(httpPort) + String(httpEndpoint);

/*
void WiFiOn() {
  wifi_fpm_do_wakeup();
  wifi_fpm_close();
  wifi_set_opmode(STATION_MODE);
  wifi_station_connect();
}

void WiFiOff() {
  wifi_station_disconnect();
  wifi_set_opmode(NULL_MODE);
  wifi_set_sleep_type(MODEM_SLEEP_T);
  wifi_fpm_open();
  wifi_fpm_do_sleep(0xFFFFFFF);
}

void light_sleep(){
   wifi_station_disconnect();
   wifi_set_opmode_current(NULL_MODE);
   wifi_fpm_set_sleep_type(LIGHT_SLEEP_T); // set sleep type, the above    posters wifi_set_sleep_type() didnt seem to work for me although it did let me compile and upload with no errors 
   wifi_fpm_open(); // Enables force sleep
   gpio_pin_wakeup_enable(GPIO_ID_PIN(2), GPIO_PIN_INTR_LOLEVEL); // GPIO_ID_PIN(2) corresponds to GPIO2 on ESP8266-01 , GPIO_PIN_INTR_LOLEVEL for a logic low, can also do other interrupts, see gpio.h above
   wifi_fpm_do_sleep(0xFFFFFFF); // Sleep for longest possible time
}

void light_sleep_us(uint32 time){
   wifi_station_disconnect();
   wifi_set_opmode_current(NULL_MODE);
   wifi_fpm_set_sleep_type(LIGHT_SLEEP_T); // set sleep type, the above    posters wifi_set_sleep_type() didnt seem to work for me although it did let me compile and upload with no errors 
   wifi_fpm_open(); // Enables force sleep
   gpio_pin_wakeup_enable(GPIO_ID_PIN(D5), GPIO_PIN_INTR_LOLEVEL); // GPIO_ID_PIN(2) corresponds to GPIO2 on ESP8266-01 , GPIO_PIN_INTR_LOLEVEL for a logic low, can also do other interrupts, see gpio.h above
   wifi_fpm_do_sleep(time); // Sleep for longest possible time
}
*/

void WiFiOff() {
    WiFi.mode(WIFI_OFF);
    WiFi.forceSleepBegin();   // turn off ESP8266 RF
    delay(1);
}

void WiFiOn() {
    WiFi.forceSleepWake();
    delay(1);
}

void setupWiFiBlocking(void) {
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

void setup(void) {
    // Power management
    pinMode(D6, OUTPUT); // Power OFF CMD
    digitalWrite(D6, LOW); // Maintain Power ON

    Serial.begin(115200);
    Serial.println();

    u8g2.setFont(u8g2_font_5x7_tf);

    String message = "Setting up WiFi on " + String(ssid) + " ...";
    webAnimator.displayText(message);

    setupWiFiBlocking();

    Serial.println(F(""));

    // print the SSID of the network you're attached to:
    Serial.print(F("SSID: "));
    Serial.println(WiFi.SSID());

    // print your WiFi shield's IP address:
    IPAddress ip = WiFi.localIP();
    Serial.print(F("IP Address: "));
    Serial.println(ip);
}

void loop(void) {
    

    webAnimator.displayText(F("HTTP query ..."));

    webAnimator.requestAnimation(webpage); //Specify the URL

    WiFiOff();

    webAnimator.displayAnimationOnce();

    delay(3000);

    pinMode(D6, INPUT); // Power OFF

    //ESP.deepSleep(10000000);
    //ESP.restart();
}

