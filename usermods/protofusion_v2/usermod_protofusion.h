//
// protofusion usermod
//
//  - OSC server sends messages to specified endpoint
//  - Analog input (optionally controls intensty and / or brightness)
//  - Digital inputs 
//
// NOTE: Strapping pins are
// - GPIO0 (internal PU)
// - GPIO2 (internal PD)
// - GPIO4 (internal PD)
// - GPIO5 (internal PU)
// - GPIO15 (internal PU)


#pragma once

#include "wled.h"
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <MicroOscUdp.h>
#include <WiFiUdp.h>


// the default frequency to read the analog distance sensor (ms)
#ifndef USERMOD_PROTOFUSION_MEASUREMENT_INTERVAL
  #define USERMOD_PROTOFUSION_MEASUREMENT_INTERVAL 1000
#endif

// how many seconds after boot to take first measurement, 10 seconds
#ifndef USERMOD_PROTOFUSION_FIRST_MEASUREMENT_AT
  #define USERMOD_PROTOFUSION_FIRST_MEASUREMENT_AT 5000
#endif


class Usermod_Protofusion : public Usermod
{
private:
  // If we've connected to WIFI and set up OSC
  uint8_t isConnected = 0;

  unsigned long readingInterval = USERMOD_PROTOFUSION_MEASUREMENT_INTERVAL;
  unsigned long lastMeasurement = UINT32_MAX - (USERMOD_PROTOFUSION_MEASUREMENT_INTERVAL - USERMOD_PROTOFUSION_FIRST_MEASUREMENT_AT);

  float lastReading = -1.0f;
  float avg_reading = 0.0f;

  // flag set at startup
  bool enabled = false;
  bool distCtlBrightness = true;
  bool distCtlIntensity = true;

  // strings to reduce flash memory usage (used more than twice)
  static const char _name[];
  static const char _enabled[];
  static const char _readInterval[];
  static const char _distance_controls_brightness[];
  static const char _distance_controls_intensity[];
  static const char _distance_sensor_pin[];
  static const char _digital0_pin[];
  static const char _digital1_pin[];
  static const char _digital2_pin[];
  static const char _osc_destination_ip1[];
  static const char _osc_destination_ip2[];
  static const char _osc_destination_ip3[];
  static const char _osc_destination_ip4[];

  // Default destination IP, changeable from web interface 
  uint8_t osc_dest_ip[4] = {192, 168, 1, 22};

  // Default pin for dist sensor. -1 is disabled.
  int8_t distance_sensor_pin = 32;
  int8_t digital0_pin = -1;
  int8_t digital1_pin = -1;
  int8_t digital2_pin = -1;

  // Ports for OSC
  unsigned int osc_rx_port = 8888;
  unsigned int osc_tx_port = 5005;

  Adafruit_SSD1306* display;
  WiFiUDP osc_udp;
  MicroOscUdp<1024>* osc;

  uint16_t segment_stop = 0; // emz testing


public:
  void setup()
  {
    display = new Adafruit_SSD1306(128, 32, &Wire, -1);
    IPAddress tx_ip = IPAddress(osc_dest_ip[0], osc_dest_ip[1], osc_dest_ip[2], osc_dest_ip[3]);
    osc = new MicroOscUdp<1024>(&osc_udp, tx_ip, osc_tx_port);

    // Allocate pins
    PinManager::allocatePin(distance_sensor_pin, false, PinOwner::UM_PROTOFUSION);
    
    if(digital0_pin != -1)
    {
      PinManager::allocatePin(digital0_pin, false, PinOwner::UM_PROTOFUSION);
      pinMode(digital0_pin, INPUT_PULLUP);
    }
    if(digital1_pin != -1)
    {
        PinManager::allocatePin(digital1_pin, false, PinOwner::UM_PROTOFUSION);
        pinMode(digital1_pin, INPUT_PULLUP);
    }
    if(digital2_pin != -1)
    {
      PinManager::allocatePin(digital2_pin, false, PinOwner::UM_PROTOFUSION);
      pinMode(digital2_pin, INPUT_PULLUP);
    }

    if(distance_sensor_pin != -1)
    {
      // set pinmode
      pinMode(distance_sensor_pin, INPUT);
    }

    if(!display->begin(SSD1306_SWITCHCAPVCC, 0x3C)) // See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
    { 
        DEBUG_PRINTF("protofusion: LCD didn't init....");
        // uh oh
    }
    else
    {
        display->clearDisplay();
        display->display();
        display->setTextColor(SSD1306_WHITE);
        display->setCursor(10, 0);
        display->println(F("Ethernet Connecting"));
        display->display();

        uint8_t minSegmentId = strip.getMainSegmentId();
        Segment &seg = strip.getSegment(minSegmentId);

        segment_stop= seg.stop;
    }

  }

  void loop()
  {
    if (!enabled || strip.isUpdating())
      return;

    if(isConnected == 0)
    {
      if(WLED_CONNECTED)
      {
          // set up osc
          osc_udp.begin(osc_rx_port);
          isConnected = 1;

          display->clearDisplay();
          display->setTextColor(SSD1306_WHITE);
          display->setCursor(10, 0);
          display->println(F("Ethernet Connected"));
          display->setTextSize(1); // Draw 2X-scale text
          display->println(ETH.localIP().toString());
          display->display();      // Show initial text
      }
      else{
        return;
      }
    }

    unsigned long now = millis();
 
    if (now - lastMeasurement > readingInterval)
    {    
      lastMeasurement = now;

      if(distance_sensor_pin != -1)
      {
        lastReading = analogRead(distance_sensor_pin) / 4096.0;
        avg_reading = avg_reading * 0.6f + lastReading * 0.4f;
        osc->sendFloat("/distance", lastReading);
      }
      if(digital0_pin != -1)
        osc->sendInt("/digital0", digitalRead(digital0_pin));
      if(digital1_pin != -1)
        osc->sendInt("/digital1", digitalRead(digital1_pin));
      if(digital2_pin != -1)
        osc->sendInt("/digital2", digitalRead(digital2_pin));


      if(distCtlBrightness)
      {
        strip.setBrightness(avg_reading*255, false); // update brightness;  immediately redraw
      }
      if(distCtlIntensity)
      {
        strip.getSegment(0).intensity = (avg_reading-0.12)*128.0f*1.12f;

        /// Sketchy testing /////////////////////////////////
        uint8_t minSegmentId = strip.getMainSegmentId();
        Segment &seg = strip.getSegment(minSegmentId);
        if (seg.isActive()) 
        {
              //seg.setOption(SEG_OPTION_ON, true);
              //seg.stop = lastReading * segment_stop; // emz can we do this??
        }
      }

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
    top[FPSTR(_enabled)] = enabled;
    top[FPSTR(_readInterval)] = readingInterval;
    top[FPSTR(_distance_controls_brightness)] = distCtlBrightness;
    top[FPSTR(_distance_controls_intensity)] = distCtlIntensity;
    top[FPSTR(_distance_sensor_pin)] = distance_sensor_pin;
    top[FPSTR(_digital0_pin)] = digital0_pin;
    top[FPSTR(_digital1_pin)] = digital1_pin;
    top[FPSTR(_digital2_pin)] = digital2_pin;
    top[FPSTR(_osc_destination_ip1)] = osc_dest_ip[0];
    top[FPSTR(_osc_destination_ip2)] = osc_dest_ip[1];
    top[FPSTR(_osc_destination_ip3)] = osc_dest_ip[2];
    top[FPSTR(_osc_destination_ip4)] = osc_dest_ip[3];
    DEBUG_PRINTLN(F("Protofusion config saved."));
  }

  /**
  * readFromConfig() is called before setup() to populate properties from values stored in cfg.json
  */
  bool readFromConfig(JsonObject &root)
  {
    JsonObject top = root[FPSTR(_name)];

    bool configComplete = !top.isNull();

    configComplete &= getJsonValue(top[FPSTR(_enabled)], enabled);
    configComplete &= getJsonValue(top[FPSTR(_readInterval)], readingInterval);
    configComplete &= getJsonValue(top[FPSTR(_distance_controls_brightness)], distCtlBrightness);
    configComplete &= getJsonValue(top[FPSTR(_distance_controls_intensity)], distCtlIntensity);

    configComplete &= getJsonValue(top[FPSTR(_distance_sensor_pin)], distance_sensor_pin);
    configComplete &= getJsonValue(top[FPSTR(_digital0_pin)], digital0_pin);
    configComplete &= getJsonValue(top[FPSTR(_digital1_pin)], digital1_pin);
    configComplete &= getJsonValue(top[FPSTR(_digital2_pin)], digital2_pin);

    configComplete &= getJsonValue(top[FPSTR(_osc_destination_ip1)], osc_dest_ip[0]);
    configComplete &= getJsonValue(top[FPSTR(_osc_destination_ip2)], osc_dest_ip[1]);
    configComplete &= getJsonValue(top[FPSTR(_osc_destination_ip3)], osc_dest_ip[2]);
    configComplete &= getJsonValue(top[FPSTR(_osc_destination_ip4)], osc_dest_ip[3]);


    // "pin" fields have special handling in settings page (or some_pin as well)
    // configComplete &= getJsonValue(top["pin"][0], testPins[0], -1);
    // configComplete &= getJsonValue(top["pin"][1], testPins[1], -1);

    return configComplete;

    // use "return !top["newestParameter"].isNull();" when updating Usermod with new features
    // return true;
  }
};

// strings to reduce flash memory usage (used more than twice)
const char Usermod_Protofusion::_name[] PROGMEM = "protofusion_v8";
const char Usermod_Protofusion::_enabled[] PROGMEM = "enabled";
const char Usermod_Protofusion::_readInterval[] PROGMEM = "distance-interval-ms";
const char Usermod_Protofusion::_distance_controls_brightness[] PROGMEM = "distance-sets-brightness";
const char Usermod_Protofusion::_distance_controls_intensity[] PROGMEM = "distance-sets-intensity";
const char Usermod_Protofusion::_distance_sensor_pin[] PROGMEM = "distance-sensor-pin";
const char Usermod_Protofusion::_digital0_pin[] PROGMEM = "pin-digital-input-0_pin";
const char Usermod_Protofusion::_digital1_pin[] PROGMEM = "pin-digital-input-1_pin";
const char Usermod_Protofusion::_digital2_pin[] PROGMEM = "pin-digital-input-2_pin";
const char Usermod_Protofusion::_osc_destination_ip1[] PROGMEM = "osc-destination-ip-1";
const char Usermod_Protofusion::_osc_destination_ip2[] PROGMEM = "osc-destination-ip-2";
const char Usermod_Protofusion::_osc_destination_ip3[] PROGMEM = "osc-destination-ip-3";
const char Usermod_Protofusion::_osc_destination_ip4[] PROGMEM = "osc-destination-ip-4";
