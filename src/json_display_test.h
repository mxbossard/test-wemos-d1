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


void setup(void) {
    Serial.begin(115200);

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

            delay(3000);
        }
    }
    
    //const char* text = doc["pages"][0]["text"];
    

    // deley between each page
    delay(3000);

}