#include <Arduino.h>
#include <U8g2lib.h>

#include <ArduinoJson.h>
#include <Wire.h>

U8G2_SSD1306_64X48_ER_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE, 5, 4);   // EastRising 0.66" OLED breakout board, Uno: A4=SDA, A5=SCL, 5V powered

char input[] = "{\"name\":\"ArduinoJson\",\"stargazers\":{""\"totalCount\":4241},\"issues\":{\"totalCount\":12}}";
const char jsonPayload[] = "{\"pages\": [{\"text\": \"foo bar\"}, {\"text\": \"Lorem ipsum\"}, {\"text\": \"Lorem ipsum dolor\"}, {\"text\": \"Lorem ipsum dolor sit amet\"}, {\"text\": \"Lorem ipsum dolor sit amet, consectetur adipiscing elit.\"}, {\"text\": \"foo baz\"}, {\"text\": \"ploof\"}]}";

// Enough space for://  + 1 object with 1 member//  + 4 objects with 1 member
const uint8_t capacity = JSON_OBJECT_SIZE(1) + 4 * JSON_OBJECT_SIZE(1) + 10 * JSON_OBJECT_SIZE(1);

void setup(void) {
    Serial.begin(115200);

    u8g2.begin();
    //u8g2.setFont(u8g2_font_micro_tr);
    //u8g2.setFont(u8g2_font_4x6_tf);
    //u8g2.setFont(u8g2_font_5x7_tf);
    u8g2.setFont(u8g2_font_5x8_tf);
    //u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.setFontRefHeightExtendedText();
    u8g2.setDrawColor(1);
    u8g2.setFontPosTop();
    u8g2.setFontDirection(0);
    //u8g2.enableUTF8Print();
}

void loop(void) {
    // picture loop

    
    DynamicJsonDocument doc(2048);
    //DynamicJsonBuffer jsonBuffer(bufferSize);
    DeserializationError err = deserializeJson(doc, jsonPayload);
    if(err) {
        Serial.print(F("deserializeJson() failed with code "));
        Serial.println(err.c_str());
    }

    JsonArray pages = doc["pages"];
    if (pages) {
        // Walk the JsonArray efficiently
        for(JsonObject page: pages) {
            const char* text = page["text"];

            Serial.print(F("Will display: "));
            Serial.println(text);

            u8g2.clear();

            //u8g2.drawStr(0, 0, text);

            // Code snippet : wrap a string to display
            byte i, y;
            // u8x8 does not wrap lines.
            y = 0;
            for (i = 0; i < strlen(text); i++) {
                if ((i % 16)==0 && i!=0) y++;
                u8g2.setCursor(i % 16 * 6, y * 9);
                u8g2.print(text[i]);
                //Serial.println(text[i]);
            }

            u8g2.sendBuffer();

            delay(2000);
        }
    }
    
    //const char* text = doc["pages"][0]["text"];
    

    // deley between each page
    delay(10000);

}