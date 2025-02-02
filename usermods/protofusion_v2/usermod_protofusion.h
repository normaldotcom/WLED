#pragma once

#include "wled.h"
#include <Arduino.h>

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>


#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
  Adafruit_SSD1306 display(128, 32, &Wire, -1);


#include <WiFiUdp.h>
WiFiUDP myUdp;
unsigned int myReceivePort = 8888;
IPAddress mySendIp(192, 168, 1, 22);
unsigned int mySendPort = 5005;

#include <MicroOscUdp.h>


//1024 byte buffer for incoming messages. Maybe downsize this.
MicroOscUdp<1024> myOsc(&myUdp, mySendIp, mySendPort);


#ifndef DISTANCE_SENSOR_PIN
  #define DISTANCE_SENSOR_PIN 32 // this pin IO1
#endif

// the default frequency to read the analog distance sensor (ms)
#ifndef USERMOD_PROTOFUSION_MEASUREMENT_INTERVAL
  #define USERMOD_PROTOFUSION_MEASUREMENT_INTERVAL 10000
#endif

// how many seconds after boot to take first measurement, 10 seconds
#ifndef USERMOD_PROTOFUSION_FIRST_MEASUREMENT_AT
  #define USERMOD_PROTOFUSION_FIRST_MEASUREMENT_AT 10000
#endif


class Usermod_Protofusion : public Usermod
{
private:
  // If we've connected to WIFI and set up OSC
  uint8_t isConnected = 0;

  unsigned long readingInterval = USERMOD_PROTOFUSION_MEASUREMENT_INTERVAL;
  unsigned long lastMeasurement = UINT32_MAX - (USERMOD_PROTOFUSION_MEASUREMENT_INTERVAL - USERMOD_PROTOFUSION_FIRST_MEASUREMENT_AT);

  float lastReading = -1.0f;

  // flag set at startup
  bool disabled = false;
  bool distCtlBrightness = true;
  bool distCtlIntensity = true;

  // strings to reduce flash memory usage (used more than twice)
  static const char _name[];
  static const char _enabled[];
  static const char _readInterval[];
  static const char _distance_controls_brightness[];
  static const char _distance_controls_intensity[];


public:
  void setup()
  {
    // set pinmode
    pinMode(DISTANCE_SENSOR_PIN, INPUT);

    if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
      //Serial.println(F("SSD1306 allocation failed"));
      //for(;;); // Don't proceed, loop forever
        DEBUG_PRINTF("[protofusion] LCD didn't init....");

      // uh oh
    }
    else
    {
        display.clearDisplay();
        display.display();
        DEBUG_PRINTF("[protofusion] LCD should be doing stuff....");
        display.setTextColor(SSD1306_WHITE);
        display.setCursor(10, 0);
        display.println(F("Ethernet Connecting"));
        display.display();

    }



  }

  void loop()
  {
    if (disabled || strip.isUpdating())
      return;

    if(isConnected == 0)
    {
      if(WLED_CONNECTED)
      {
          // set up osc
          myUdp.begin(myReceivePort);
          isConnected = 1;

          display.clearDisplay();
          display.setTextColor(SSD1306_WHITE);
          display.setCursor(10, 0);
          display.println(F("Ethernet Connected"));
          display.setTextSize(2); // Draw 2X-scale text
          display.println(ETH.localIP().toString());
          display.display();      // Show initial text
      }
      else{
        return;
      }
    }

    unsigned long now = millis();

    // check to see if we are due for taking a measurement
    // lastMeasurement will not be updated until the conversion
    // is complete the the reading is finished
    if (now - lastMeasurement < readingInterval)
    {
      return;
    }

    lastReading = analogRead(DISTANCE_SENSOR_PIN) / 4096.0;
    myOsc.sendFloat("/distance", lastReading);
    if(distCtlBrightness)
    {
      strip.setBrightness(lastReading*255, false); // update brightness;  immediately redraw
    }
    if(distCtlIntensity)
    {
      strip.getSegment(0).intensity = lastReading*128;
    }
  }

  void addToJsonInfo(JsonObject &root)
  {
    JsonObject user = root[F("protofusion")];
    if (user.isNull())
      user = root.createNestedObject(F("protofusion"));

    JsonArray dist = user.createNestedArray(F("distance"));

    dist.add(lastReading);
  }

  uint16_t getId()
  {
    return USERMOD_ID_PROTOFUSION;
  }

  /**
     * addToConfig() (called from set.cpp) stores persistent properties to cfg.json
     */
  void addToConfig(JsonObject &root)
  {
    // we add JSON object.
    JsonObject top = root.createNestedObject(FPSTR(_name)); // usermodname
    top[FPSTR(_enabled)] = !disabled;
    top[FPSTR(_readInterval)] = readingInterval;
    top[FPSTR(_distance_controls_brightness)] = distCtlBrightness;
    top[FPSTR(_distance_controls_intensity)] = distCtlIntensity;
    
    DEBUG_PRINTLN(F("Protofusion config saved."));
  }

  /**
  * readFromConfig() is called before setup() to populate properties from values stored in cfg.json
  */
  bool readFromConfig(JsonObject &root)
  {
    // we look for JSON object.
    JsonObject top = root[FPSTR(_name)];
    if (top.isNull()) {
      DEBUG_PRINT(FPSTR(_name));
      DEBUG_PRINTLN(F(": No config found. (Using defaults.)"));
      return false;
    }

    disabled         = !(top[FPSTR(_enabled)] | !disabled);
    readingInterval  = (top[FPSTR(_readInterval)] | readingInterval/1000); // convert to ms
    distCtlBrightness = (top[FPSTR(_distance_controls_brightness)] | !_distance_controls_brightness);
    distCtlIntensity = (top[FPSTR(_distance_controls_intensity)] | !_distance_controls_intensity);
    DEBUG_PRINT(FPSTR(_name));
    DEBUG_PRINTLN(F(" config (re)loaded."));

    // use "return !top["newestParameter"].isNull();" when updating Usermod with new features
    return true;
  }
};

// strings to reduce flash memory usage (used more than twice)
const char Usermod_Protofusion::_name[] PROGMEM = "protofusion";
const char Usermod_Protofusion::_enabled[] PROGMEM = "enabled";
const char Usermod_Protofusion::_readInterval[] PROGMEM = "distance-interval-ms";
const char Usermod_Protofusion::_distance_controls_brightness[] PROGMEM = "distance-sets-brightness";
const char Usermod_Protofusion::_distance_controls_intensity[] PROGMEM = "distance-sets-intensity";
