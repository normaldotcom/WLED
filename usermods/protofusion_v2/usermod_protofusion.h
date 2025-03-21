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
#include "src/dependencies/e131/ESPAsyncE131.h"
#include <RemoteDebug.h>
#include <Adafruit_TCA8418.h>
#include <Adafruit_seesaw.h>

// the default frequency to read the analog distance sensor (ms)
#ifndef USERMOD_PROTOFUSION_MEASUREMENT_INTERVAL
  #define USERMOD_PROTOFUSION_MEASUREMENT_INTERVAL 100
#endif

// how many seconds after boot to take first measurement, 10 seconds
#ifndef USERMOD_PROTOFUSION_FIRST_MEASUREMENT_AT
  #define USERMOD_PROTOFUSION_FIRST_MEASUREMENT_AT 5000
#endif


#define PROTOFUSION_ARTNET_PORT 6454
//from e131
#define MAX_3_CH_LEDS_PER_UNIVERSE 170
#define MAX_4_CH_LEDS_PER_UNIVERSE 128
#define MAX_CHANNELS_PER_UNIVERSE 512

#define NUM_ANALOG_MODS 2

typedef struct _analog_mod_s_
{
  int8_t analog_input = -1;
  bool osc_input = false;
  float min_value = 0.0f;
  float max_value = 1.0f;
  uint8_t segment_id = 0;
  bool set_brightness = false;
  bool set_intensity = false;
  bool modulate_artnet = false;
} analog_mod_t;

typedef struct _analog_mod_global_s_
{
  float lastReading = -1.0f;
  float lastSentReading = -1.0f;
  float avg_reading = 0.0f;
  float osc_value = 0.0f;
  bool osc_artnet_enable = false;
  bool analog_artnet_enable = false;

  uint32_t segment_id = 0;
} analog_mod_global_t;



static analog_mod_global_t analog_mod_global[NUM_ANALOG_MODS];

static int8_t digital_out0_pin_global = -1;

// Private Prototypes
void handleArtnetPollReplyEMZ(IPAddress ipAddress);
void sendArtnetPollReplyEMZ(ArtPollReply *reply, IPAddress ipAddress, uint16_t portAddress);
void handleE131PacketEMZ(e131_packet_t* p, IPAddress clientIP, byte protocol);
static void osc_parser( MicroOscMessage& receivedOscMessage);
static bool find_next_led(uint16_t* current_strip, int* current_led_on_strip);


// Private Variables
RemoteDebug Debug;
ESPAsyncE131 secondary_e131(handleE131PacketEMZ);
Adafruit_TCA8418 tio;
Adafruit_seesaw ss;



class Usermod_Protofusion : public Usermod
{
private:
  // If we've connected to WIFI and set up OSC
  uint8_t isConnected = 0;

  unsigned long readingInterval = USERMOD_PROTOFUSION_MEASUREMENT_INTERVAL;
  unsigned long lastMeasurement = UINT32_MAX - (USERMOD_PROTOFUSION_MEASUREMENT_INTERVAL - USERMOD_PROTOFUSION_FIRST_MEASUREMENT_AT);
  unsigned long lastHeartbeat = 0;

  // flag set at startup
  bool enabled = true;
  bool send_on_change = true;

  // strings to reduce flash memory usage (used more than twice)
  static const char _name[];
  static const char _enabled[];
  static const char _readInterval[];
  static const char _send_on_change[];
  static const char _digital0_pin[];
  static const char _digital1_pin[];
  static const char _digital2_pin[];

  static const char _digital_out0_pin[];

  static const char _osc_destination_ip1[];
  static const char _osc_destination_ip2[];
  static const char _osc_destination_ip3[];
  static const char _osc_destination_ip4[];

  static const char _mod1_analog_input[];
  static const char _mod1_osc_input[];
  static const char _mod1_min_value[];
  static const char _mod1_max_value[];
  static const char _mod1_segment_id[];
  static const char _mod1_set_brightness[];
  static const char _mod1_set_intensity[];
  static const char _mod1_modulate_artnet[];

  static const char _mod2_analog_input[];
  static const char _mod2_osc_input[];
  static const char _mod2_min_value[];
  static const char _mod2_max_value[];
  static const char _mod2_segment_id[];
  static const char _mod2_set_brightness[];
  static const char _mod2_set_intensity[];
  static const char _mod2_modulate_artnet[];


  analog_mod_t analog_mod[NUM_ANALOG_MODS]; // use default values, thanks c++!

  bool gpio_expander_connected = false;
  bool encoder_connected = false;

  // Default destination IP, changeable from web interface 
  uint8_t osc_dest_ip[4] = {192, 168, 1, 22};

  // Default pin for dist sensor. -1 is disabled.
  int8_t digital0_pin = -1;
  int8_t digital0_pin_val = -1;
  int8_t digital1_pin = -1;
  int8_t digital1_pin_val = -1;
  int8_t digital2_pin = -1;
  int8_t digital2_pin_val = -1;
  int8_t digital_out0_pin = -1;

  // Ports for OSC
  unsigned int osc_rx_port = 5000;
  unsigned int osc_tx_port = 5005;

  Adafruit_SSD1306* display;
  WiFiUDP osc_udp;
  MicroOscUdp<1024>* osc;


public:
  void setup()
  {
    display = new Adafruit_SSD1306(128, 32, &Wire, -1);
    IPAddress tx_ip = IPAddress(osc_dest_ip[0], osc_dest_ip[1], osc_dest_ip[2], osc_dest_ip[3]);
    osc = new MicroOscUdp<1024>(&osc_udp, tx_ip, osc_tx_port);

    // Attempt to talk to GPIO expander
    // Wire.beginTransmission(0x34);
    // if(Wire.endTransmission() == 0)
    // {
    //   gpio_expander_connected = true;
    // }

    if (! tio.begin(TCA8418_DEFAULT_ADDR, &Wire)) 
    {
      gpio_expander_connected = false;
    }
    else
    {
      gpio_expander_connected = true;

      // Enable debounce
      tio.enableDebounce();

      // Init pins
      for (uint8_t pin = 0; pin < 18; pin++)
      {
        tio.pinMode(pin, INPUT_PULLUP);

        // Interrupts - TESTING
        tio.pinIRQMode(pin, FALLING);
      }

      pinMode(13, INPUT_PULLUP); // EMZ TESTING

      //  flush pending interrupts
      tio.flush();
      //  enable interrupt mode
      tio.enableInterrupts();
    }



    if (! ss.begin(0x36)) 
    {
      encoder_connected = false;
    }
    else
    {
      encoder_connected = true;
    }
    
    // Set up each analog mod
    for(uint8_t i=0; i<NUM_ANALOG_MODS; i++)
    {
      // Allocate pins
      if(analog_mod[i].analog_input != -1)
      {
        PinManager::allocatePin(analog_mod[i].analog_input, false, PinOwner::UM_PROTOFUSION);
        pinMode(analog_mod[i].analog_input, INPUT);
        DEBUG_PRINTF("protofusion: allocated mod%d analog input %d", i, analog_mod[i].analog_input);

      }
    }
    
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
    if(digital_out0_pin != -1)
    {
      PinManager::allocatePin(digital_out0_pin, true, PinOwner::UM_PROTOFUSION);
      pinMode(digital_out0_pin, OUTPUT);
      digital_out0_pin_global = digital_out0_pin;
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
        display->println(F("LumaPXL: Connecting"));
        display->display();
    }

  }


  void loop()
  {
    
    char osc_path[128] = {0};

    if (!enabled || strip.isUpdating())
      return;

    if(isConnected == 0)
    {
      if(ETH.localIP()[0]) //WLED_CONNECTED)
      {
          Debug.begin("DebugHOST"); // Initialize the WiFi server
          Debug.setResetCmdEnabled(true); // Enable the reset command
          Debug.showProfiler(true); // Profiler (Good to measure times, to optimize codes)
          Debug.showColors(true); // Colors

          // set up osc
          osc_udp.begin(osc_rx_port);
          isConnected = 1;

          display->clearDisplay();
          display->setTextColor(SSD1306_WHITE);
          display->setCursor(0, 0);
          display->println(F("LumaPXL: Ready     "));
          display->setTextSize(1); // Draw 2X-scale text
          //display->println("");
          display->print("  IP: ");
          display->println(ETH.localIP().toString());

          if(gpio_expander_connected)
            display->println("  Xpand: OK ");
          else
            display->println("  Xpnd: Offline");

          if(encoder_connected)
            display->println("   Enc: OK ");
          else
            display->println("   Enc: Offline");

          display->display();      // Show initial text

          
        bool success = secondary_e131.begin(false, PROTOFUSION_ARTNET_PORT, 1, 5); //E131_MAX_UNIVERSE_COUNT);
        if(success)
          DEBUG_PRINTLN(F("Protofusion: e131 init completed OK."));
        else
          DEBUG_PRINTLN(F("Protofusion: e131 init failed."));



      }
      else{
        return;
      }
    }





    //  Handle ISR
    if (gpio_expander_connected == true && digitalRead(13) == 0)
    {
      //  CHECK WHICH INTERRUPTS TO HANDLE
      int intStat = tio.readRegister(TCA8418_REG_INT_STAT);
      if (intStat & 0x02)
      {
        //  reading the registers is mandatory to clear IRQ flag
        //  can also be used to find the GPIO changed
        //  as these registers are a bitmap of the gpio pins.
        tio.readRegister(TCA8418_REG_GPIO_INT_STAT_1);
        tio.readRegister(TCA8418_REG_GPIO_INT_STAT_2);
        tio.readRegister(TCA8418_REG_GPIO_INT_STAT_3);
        //  clear GPIO IRQ flag
        tio.writeRegister(TCA8418_REG_INT_STAT, 2);
      }

      if (intStat & 0x01)
      {
        //  datasheet page 16 - Table 2
        int keyCode = tio.getEvent();
        uint8_t gpio = (keyCode & 0x7F) - 97;
        if(keyCode & 0x80)
        {
          //  map keyCode to GPIO nr.
          debugI("Xpand: Press on pin %u\r\n", gpio);
          snprintf(osc_path, 128, "/%s/digital%u", cmDNS, gpio+10); // Offset of 10 from onboard GPIO
          osc->sendInt(osc_path, 1);
        }
        else
        {
          //  map keyCode to GPIO nr.
          debugI("Xpand: Release on pin %u\r\n", gpio);
          snprintf(osc_path, 128, "/%s/digital%u", cmDNS, gpio+10); // Offset of 10 from onboard GPIO
          osc->sendInt(osc_path, 0);
        }

        //  clear the EVENT IRQ flag
        tio.writeRegister(TCA8418_REG_INT_STAT, 1);
      }

      //  check pending events
      // int intstat = tio.readRegister(TCA8418_REG_INT_STAT);
      // if ((intstat & 0x03) == 0) TCA8418_event = false;

    }

    // TODO: only send if change and send_on_change
    if(digital0_pin != -1)
    {
      uint8_t val = digitalRead(digital0_pin);
      if(digital0_pin_val != val)
      {
        snprintf(osc_path, 128, "/%s/digital0", cmDNS);
        osc->sendInt(osc_path, val);
      }
      digital0_pin_val = val;
    }
    if(digital1_pin != -1)
    {
      uint8_t val = digitalRead(digital1_pin);
      if(digital1_pin_val != val)
      {
        snprintf(osc_path, 128, "/%s/digital1", cmDNS);
        osc->sendInt(osc_path, val);
      }
      digital1_pin_val = val;
    }
    if(digital2_pin != -1)
    {
      uint8_t val = digitalRead(digital2_pin);
      if(digital2_pin_val != val)
      {
        snprintf(osc_path, 128, "/%s/digital2", cmDNS);
        osc->sendInt(osc_path, val);
      }
      digital2_pin_val = val;
    }
    
    


    unsigned long now = millis();


    if(now - lastHeartbeat > 1000)
    {
      lastHeartbeat = now;

      snprintf(osc_path, 128, "/%s/heartbeat", cmDNS);
      osc->sendInt(osc_path, 1);
    }

 
    if (now - lastMeasurement > readingInterval)
    {    
      lastMeasurement = now;
      //      debugI("Test debug print %u\r\n", lastMeasurement);

      // debugI("GPIO Expander State: %u\r\n", gpio_expander_connected);

      // debugI("Strips:\r\n");
      // for(uint8_t i=0; i<strip.getSegmentsNum(); i++)
      // {
      //   debugI("  Strip %u: %u LEDs, frozen=%u\r\n", i, strip.getSegment(i).length(), strip.getSegment(i).freeze);
      // }

      // if(encoder_connected)
      // {
      //   debugI("Encoder position: %u\r\n", ss.getEncoderPosition());
      // }

      

      Debug.handle();

      osc->onOscMessageReceived( osc_parser );



      if(encoder_connected)
      {
        int32_t pos = ss.getEncoderPosition();
        debugI("Encoder position: %d\r\n", pos);
        snprintf(osc_path, 128, "/%s/encoder", cmDNS);
        osc->sendFloat(osc_path, pos);
      }


      for(uint8_t i=0; i<NUM_ANALOG_MODS; i++)
      {
        if(analog_mod[i].osc_input == true)
        { 
          // TODO: Need to figure out how to update artnet stuff


          // Read in OSC message and set things
          if(analog_mod[i].set_brightness)
          {
            // EMZ figure out how to do this per-segment...
            strip.setBrightness(analog_mod_global[i].osc_value*255, false); // update brightness;  immediately redraw
          }
          if(analog_mod[i].set_intensity)
          {
            uint8_t intensity = (analog_mod_global[i].osc_value)*128.0f;
            strip.getSegment(analog_mod[i].segment_id).intensity = intensity;
            // DEBUG_PRINTF("mod1 intensity: %d\r\n", intensity);
          }
        }    


        else if(analog_mod[i].analog_input != -1)
        {
          // Convert ADC reading to 0-1
          float raw = analogRead(analog_mod[i].analog_input) / 4096.0;

          // Scale based on min/max specified by user
          if(raw > analog_mod[i].max_value)
            raw = analog_mod[i].max_value;
          if(raw < analog_mod[i].min_value)
            raw = analog_mod[i].min_value;
          analog_mod_global[i].lastReading = (raw - analog_mod[i].min_value) / (analog_mod[i].max_value - analog_mod[i].min_value);
          
          bool value_changed = false;
          if(fabsf(analog_mod_global[i].lastReading - analog_mod_global[i].lastSentReading) > 0.01f)
          {
            value_changed = true;
          }

          // Alpha filter of the readings
          analog_mod_global[i].avg_reading = analog_mod_global[i].avg_reading * 0.6f + analog_mod_global[i].lastReading * 0.4f;

          // Only send value if changed or if not sending on change only
          if(value_changed || send_on_change == false)
          {
            snprintf(osc_path, 128, "/%s/analog%u", cmDNS, i);
            osc->sendFloat(osc_path, analog_mod_global[i].lastReading);
            analog_mod_global[i].lastSentReading = analog_mod_global[i].lastReading;
          }

          if(analog_mod[i].set_brightness)
          {
            // EMZ figure out how to do this per-segment...
            // strip.setBrightness(analog_mod_global[i].lastReading*255, false); // update brightness;  immediately redraw
            strip.getSegment(analog_mod[i].segment_id).setOpacity(analog_mod_global[i].lastReading*255.0f);
          }
          if(analog_mod[i].set_intensity)
          {
            uint8_t intensity = (analog_mod_global[i].avg_reading)*128.0f;
            strip.getSegment(analog_mod[i].segment_id).intensity = intensity;
            // DEBUG_PRINTF("mod1 intensity: %d\r\n", intensity);

          }
          // DEBUG_PRINTF("mod1 active, pin %d reads %f converted to %f [max=%f min=%f]\r\n", mod1_analog_input, raw, mod1_lastReading, mod1_max_value, mod1_min_value);

        }
        else
        {
          // DEBUG_PRINTLN("protofusion: mod1 not active\r\n");
        }

        // Ugh. Expose stuff for the Artnet callback.
        analog_mod_global[i].segment_id = analog_mod[i].segment_id;
        analog_mod_global[i].osc_artnet_enable = analog_mod[i].osc_input;
        analog_mod_global[i].analog_artnet_enable = analog_mod[i].analog_input != -1;
      }


      

    }
  }

  void addToJsonInfo(JsonObject &root)
  {
    // JsonObject user = root[F("protofusion")];
    // if (user.isNull())
    //   user = root.createNestedObject(F("protofusion"));

    // JsonArray dist = user.createNestedArray(F("distance"));

    // dist.add(lastReading);
  }


  // EMZ so this is called right before every strip update.... 
  // Could use this to blackout pixels per strip with normally RX'ed artnet data,
  // caveat is we wouldn't be able to do simlutaneous artnet and internal effects.
  // Probably still need our own artnet server to make this happen.
  // void handleOverlayDraw() override
  // {
  //   strip.setPixelColor(5, RGBW32(0,0,0,0)); // set the first pixel to black
  // }



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
    top[FPSTR(_send_on_change)] = send_on_change;
    
    top[FPSTR(_digital0_pin)] = digital0_pin;
    top[FPSTR(_digital1_pin)] = digital1_pin;
    top[FPSTR(_digital2_pin)] = digital2_pin;

    top[FPSTR(_digital_out0_pin)] = digital_out0_pin;

    top[FPSTR(_osc_destination_ip1)] = osc_dest_ip[0];
    top[FPSTR(_osc_destination_ip2)] = osc_dest_ip[1];
    top[FPSTR(_osc_destination_ip3)] = osc_dest_ip[2];
    top[FPSTR(_osc_destination_ip4)] = osc_dest_ip[3];

    top[FPSTR(_mod1_analog_input)] = analog_mod[0].analog_input;
    top[FPSTR(_mod1_osc_input)] = analog_mod[0].osc_input;
    top[FPSTR(_mod1_min_value)] = analog_mod[0].min_value; 
    top[FPSTR(_mod1_max_value)] = analog_mod[0].max_value;
    top[FPSTR(_mod1_segment_id)] = analog_mod[0].segment_id;
    top[FPSTR(_mod1_set_brightness)] = analog_mod[0].set_brightness;
    top[FPSTR(_mod1_set_intensity)] = analog_mod[0].set_intensity;
    top[FPSTR(_mod1_modulate_artnet)] = analog_mod[0].modulate_artnet;

    top[FPSTR(_mod2_analog_input)] = analog_mod[1].analog_input;
    top[FPSTR(_mod2_osc_input)] = analog_mod[1].osc_input;
    top[FPSTR(_mod2_min_value)] = analog_mod[1].min_value;
    top[FPSTR(_mod2_max_value)] = analog_mod[1].max_value;
    top[FPSTR(_mod2_segment_id)] = analog_mod[1].segment_id;
    top[FPSTR(_mod2_set_brightness)] = analog_mod[1].set_brightness;
    top[FPSTR(_mod2_set_intensity)] = analog_mod[1].set_intensity;
    top[FPSTR(_mod2_modulate_artnet)] = analog_mod[1].modulate_artnet;


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
    configComplete &= getJsonValue(top[FPSTR(_send_on_change)], send_on_change);

    configComplete &= getJsonValue(top[FPSTR(_digital0_pin)], digital0_pin);
    configComplete &= getJsonValue(top[FPSTR(_digital1_pin)], digital1_pin);
    configComplete &= getJsonValue(top[FPSTR(_digital2_pin)], digital2_pin);

    configComplete &= getJsonValue(top[FPSTR(_digital_out0_pin)], digital_out0_pin);

    configComplete &= getJsonValue(top[FPSTR(_osc_destination_ip1)], osc_dest_ip[0]);
    configComplete &= getJsonValue(top[FPSTR(_osc_destination_ip2)], osc_dest_ip[1]);
    configComplete &= getJsonValue(top[FPSTR(_osc_destination_ip3)], osc_dest_ip[2]);
    configComplete &= getJsonValue(top[FPSTR(_osc_destination_ip4)], osc_dest_ip[3]);

    configComplete &= getJsonValue(top[FPSTR(_mod1_analog_input)], analog_mod[0].analog_input);
    configComplete &= getJsonValue(top[FPSTR(_mod1_osc_input)], analog_mod[0].osc_input);
    configComplete &= getJsonValue(top[FPSTR(_mod1_min_value)], analog_mod[0].min_value);
    configComplete &= getJsonValue(top[FPSTR(_mod1_max_value)], analog_mod[0].max_value);
    configComplete &= getJsonValue(top[FPSTR(_mod1_segment_id)], analog_mod[0].segment_id);
    configComplete &= getJsonValue(top[FPSTR(_mod1_set_brightness)], analog_mod[0].set_brightness);
    configComplete &= getJsonValue(top[FPSTR(_mod1_set_intensity)], analog_mod[0].set_intensity);
    configComplete &= getJsonValue(top[FPSTR(_mod1_modulate_artnet)], analog_mod[0].modulate_artnet);

    configComplete &= getJsonValue(top[FPSTR(_mod2_analog_input)], analog_mod[1].analog_input);
    configComplete &= getJsonValue(top[FPSTR(_mod2_osc_input)], analog_mod[1].osc_input);
    configComplete &= getJsonValue(top[FPSTR(_mod2_min_value)], analog_mod[1].min_value);
    configComplete &= getJsonValue(top[FPSTR(_mod2_max_value)], analog_mod[1].max_value);
    configComplete &= getJsonValue(top[FPSTR(_mod2_segment_id)], analog_mod[1].segment_id);
    configComplete &= getJsonValue(top[FPSTR(_mod2_set_brightness)], analog_mod[1].set_brightness);
    configComplete &= getJsonValue(top[FPSTR(_mod2_set_intensity)], analog_mod[1].set_intensity);
    configComplete &= getJsonValue(top[FPSTR(_mod2_modulate_artnet)], analog_mod[1].modulate_artnet);


    // "pin" fields have special handling in settings page (or some_pin as well)
    // configComplete &= getJsonValue(top["pin"][0], testPins[0], -1);
    // configComplete &= getJsonValue(top["pin"][1], testPins[1], -1);

    return configComplete;

    // use "return !top["newestParameter"].isNull();" when updating Usermod with new features
    // return true;
  }









};

// strings to reduce flash memory usage (used more than twice)
const char Usermod_Protofusion::_name[] PROGMEM = "protofusion_v14";
const char Usermod_Protofusion::_enabled[] PROGMEM = "enabled";
const char Usermod_Protofusion::_readInterval[] PROGMEM = "sampling-interval-ms";

const char Usermod_Protofusion::_send_on_change[] PROGMEM = "only-send-analog-on-change";

const char Usermod_Protofusion::_digital0_pin[] PROGMEM = "pin-digital-input-0_pin";
const char Usermod_Protofusion::_digital1_pin[] PROGMEM = "pin-digital-input-1_pin";
const char Usermod_Protofusion::_digital2_pin[] PROGMEM = "pin-digital-input-2_pin";
const char Usermod_Protofusion::_digital_out0_pin[] PROGMEM = "pin-digital-output-0_pin";

const char Usermod_Protofusion::_osc_destination_ip1[] PROGMEM = "osc-destination-ip-1";
const char Usermod_Protofusion::_osc_destination_ip2[] PROGMEM = "osc-destination-ip-2";
const char Usermod_Protofusion::_osc_destination_ip3[] PROGMEM = "osc-destination-ip-3";
const char Usermod_Protofusion::_osc_destination_ip4[] PROGMEM = "osc-destination-ip-4";

const char Usermod_Protofusion::_mod1_analog_input[] PROGMEM = "mod1-analog-input-pin";
const char Usermod_Protofusion::_mod1_osc_input[] PROGMEM = "mod1-osc-input-enable";
const char Usermod_Protofusion::_mod1_min_value[] PROGMEM = "mod1-min-value-0--1";
const char Usermod_Protofusion::_mod1_max_value[] PROGMEM = "mod1-max-value-0--1";
const char Usermod_Protofusion::_mod1_segment_id[] PROGMEM = "mod1-output-segment-id";
const char Usermod_Protofusion::_mod1_set_brightness[] PROGMEM = "mod1-set-brightness?";
const char Usermod_Protofusion::_mod1_set_intensity[] PROGMEM = "mod1-set-intensity?";
const char Usermod_Protofusion::_mod1_modulate_artnet[] PROGMEM = "mod1-modulate-artnet?";

const char Usermod_Protofusion::_mod2_analog_input[] PROGMEM = "mod2-analog-input-pin";
const char Usermod_Protofusion::_mod2_osc_input[] PROGMEM = "mod2-osc-input-enable";
const char Usermod_Protofusion::_mod2_min_value[] PROGMEM = "mod2-min-value-0--1";
const char Usermod_Protofusion::_mod2_max_value[] PROGMEM = "mod2-max-value-0--1";
const char Usermod_Protofusion::_mod2_segment_id[] PROGMEM = "mod2-output-segment-id";
const char Usermod_Protofusion::_mod2_set_brightness[] PROGMEM = "mod2-set-brightness?";
const char Usermod_Protofusion::_mod2_set_intensity[] PROGMEM = "mod2-set-intensity?";
const char Usermod_Protofusion::_mod2_modulate_artnet[] PROGMEM = "mod2-modulate-artnet?";




static void osc_parser( MicroOscMessage& receivedOscMessage) 
{


  if ( receivedOscMessage.checkOscAddressAndTypeTags("/mod1/value", "f") ) 
  {
    debugI("Received OSC message for mod1 value\r\n");
    analog_mod_global[0].osc_value = receivedOscMessage.nextAsFloat();
  }
  else if ( receivedOscMessage.checkOscAddressAndTypeTags("/mod2/value", "f") ) 
  {
    analog_mod_global[1].osc_value = receivedOscMessage.nextAsFloat();
  }
  else if ( receivedOscMessage.checkOscAddressAndTypeTags("/digital0/value", "i") ) 
  {
    digitalWrite(digital_out0_pin_global, receivedOscMessage.nextAsInt());
  }


  // Strip commands over OSC
  else if ( receivedOscMessage.checkOscAddressAndTypeTags("/strip/direction", "ii") ) 
  {
    uint8_t strip_id = receivedOscMessage.nextAsInt();
    strip.getSegment(strip_id).reverse = receivedOscMessage.nextAsInt();
    // OR do setOption(SEG_OPTION_REVERSED)
  }
  else if ( receivedOscMessage.checkOscAddressAndTypeTags("/strip/intensity", "if") ) 
  {
    uint8_t strip_id = receivedOscMessage.nextAsInt();
    strip.getSegment(strip_id).intensity = receivedOscMessage.nextAsFloat() * 255.0f;
  }
  else if ( receivedOscMessage.checkOscAddressAndTypeTags("/strip/freeze", "ii") ) 
  {
    debugI("Received OSC message for freeze\r\n");
    uint8_t strip_id = receivedOscMessage.nextAsInt();
    strip.getSegment(strip_id).freeze = receivedOscMessage.nextAsInt() == 1;
  }
  else if ( receivedOscMessage.checkOscAddressAndTypeTags("/strip/opacity", "if") ) 
  {
    debugI("Got opacity reading\r\n");
    uint8_t strip_id = receivedOscMessage.nextAsInt();
    //strip.getSegment(strip_id).opacity = receivedOscMessage.nextAsFloat() * 255.0f;
    strip.getSegment(strip_id).setOpacity(receivedOscMessage.nextAsFloat() * 255.0f);
    // FIXME: Could use setOpacity to apply this with fade transition
  }
  else if ( receivedOscMessage.checkOscAddressAndTypeTags("/strip/effect", "ii") ) 
  {
    uint8_t strip_id = receivedOscMessage.nextAsInt();
    strip.getSegment(strip_id).setMode(receivedOscMessage.nextAsInt());
  }

  else if ( receivedOscMessage.checkOscAddressAndTypeTags("/strip/color1", "ii") ) 
  {
    uint8_t strip_id = receivedOscMessage.nextAsInt();
    strip.getSegment(strip_id).setColor(0, receivedOscMessage.nextAsInt()); // This hopefully will work--32bit RGB
  }

  else if ( receivedOscMessage.checkOscAddressAndTypeTags("/strip/color2", "ii") ) 
  {
    uint8_t strip_id = receivedOscMessage.nextAsInt();
    strip.getSegment(strip_id).setColor(1, receivedOscMessage.nextAsInt()); // This hopefully will work--32bit RGB
  }
  else if ( receivedOscMessage.checkOscAddressAndTypeTags("/strip/color3", "ii") ) 
  {
    uint8_t strip_id = receivedOscMessage.nextAsInt();
    strip.getSegment(strip_id).setColor(2, receivedOscMessage.nextAsInt()); // This hopefully will work--32bit RGB
  }

  else if ( receivedOscMessage.checkOscAddressAndTypeTags("/strip/palette", "ii") ) 
  {
    uint8_t strip_id = receivedOscMessage.nextAsInt();
    strip.getSegment(strip_id).setPalette(receivedOscMessage.nextAsInt()); // 8bit palette index
  }

  else if ( receivedOscMessage.checkOscAddressAndTypeTags("/strip/speed", "if") ) 
  {
    uint8_t strip_id = receivedOscMessage.nextAsInt();
    strip.getSegment(strip_id).speed = receivedOscMessage.nextAsFloat() * 255.0f; // 8bit palette index
  }

}




//E1.31 and Art-Net protocol support
void handleE131PacketEMZ(e131_packet_t* p, IPAddress clientIP, byte protocol){
  int uni = 0, dmxChannels = 0;
  uint8_t* e131_data = nullptr;
  int seq = 0, mde = REALTIME_MODE_E131;

  if (protocol == P_ARTNET)
  {
    // DEBUG_PRINTLN(F("Protofusion: got artnet packet."));

    if (p->art_opcode == ARTNET_OPCODE_OPPOLL) {
      //DEBUG_PRINTLN(F(" artnet poll."));
      handleArtnetPollReplyEMZ(clientIP);
      return;
    }
    
    uni = p->art_universe;
    dmxChannels = htons(p->art_length);
    e131_data = p->art_data;
    seq = p->art_sequence_number;
    mde = REALTIME_MODE_ARTNET;
    // debugI(" artnet realtime uni=%d chans=%d seq=%d\r\n", uni, dmxChannels, seq);

  } else if (protocol == P_E131) {
    // Ignore PREVIEW data (E1.31: 6.2.6)
    if ((p->options & 0x80) != 0) return;
    dmxChannels = htons(p->property_value_count) - 1;
    // DMX level data is zero start code. Ignore everything else. (E1.11: 8.5)
    if (dmxChannels == 0 || p->property_values[0] != 0) return;
    uni = htons(p->universe);
    e131_data = p->property_values;
    seq = p->sequence_number;
    if (e131Priority != 0) {
      if (p->priority < e131Priority ) return;
      // track highest priority & skip all lower priorities
      if (p->priority >= highPriority.get()) highPriority.set(p->priority);
      if (p->priority < highPriority.get()) return;
    }
  } else { //DDP
    // EMZ do nothing
          DEBUG_PRINTLN(F(" DDP packet, not handled."));
    return;
  }

  // only listen for universes we're handling & allocated memory
  if (uni < e131Universe || uni >= (e131Universe + E131_MAX_UNIVERSE_COUNT)) 
  {
    DEBUG_PRINTLN(F(" uni out of bounds."));
    return;
  }

  unsigned previousUniverses = uni - e131Universe;

  if (e131SkipOutOfSequence)
    if (seq < e131LastSequenceNumber[previousUniverses] && seq > 20 && e131LastSequenceNumber[previousUniverses] < 250){
      DEBUG_PRINTF_P(PSTR("skipping E1.31 frame (last seq=%d, current seq=%d, universe=%d)\n"), e131LastSequenceNumber[previousUniverses], seq, uni);
      return;
    }
  e131LastSequenceNumber[previousUniverses] = seq;

  // update status info
  realtimeIP = clientIP;
  byte wChannel = 0;
  unsigned totalLen = strip.getLengthTotal();
  unsigned availDMXLen = 0;
  unsigned dataOffset = DMXAddress;

  // For legacy DMX start address 0 the available DMX length offset is 0
  const unsigned dmxLenOffset = (DMXAddress == 0) ? 0 : 1;

  // Check if DMX start address fits in available channels
  if (dmxChannels >= DMXAddress) {
    availDMXLen = (dmxChannels - DMXAddress) + dmxLenOffset;
  }

  // DMX data in Art-Net packet starts at index 0, for E1.31 at index 1
  if (protocol == P_ARTNET && dataOffset > 0) {
    dataOffset--;
  }

  switch (DMXMode) {
    case DMX_MODE_DISABLED:
          //DEBUG_PRINTLN(F(" dmx disabled."));
      return;  // nothing to do
      break;

    case DMX_MODE_SINGLE_RGB:   // 3 channel: [R,G,B]
    case DMX_MODE_SINGLE_DRGB:  // 4 channel: [Dimmer,R,G,B]
    case DMX_MODE_PRESET:       // 2 channel: [Dimmer,Preset]
    case DMX_MODE_EFFECT:           // 15 channels [bri,effectCurrent,effectSpeed,effectIntensity,effectPalette,effectOption,R,G,B,R2,G2,B2,R3,G3,B3]
    case DMX_MODE_EFFECT_W:         // 18 channels, same as above but with extra +3 white channels [..,W,W2,W3]
    case DMX_MODE_EFFECT_SEGMENT:   // 15 channels per segment;
    case DMX_MODE_EFFECT_SEGMENT_W: // 18 Channels per segment;
     //DEBUG_PRINTLN(F("Protofusion: got unsupported artnet packet."));
      return;
      break; // unsupported
      
    case DMX_MODE_MULTIPLE_DRGB:
    case DMX_MODE_MULTIPLE_RGB:
    case DMX_MODE_MULTIPLE_RGBW:
      {
        //DEBUG_PRINTLN(F("Protofusion: got artnet RGBW/RGB/DRGB data."));
        bool is4Chan = (DMXMode == DMX_MODE_MULTIPLE_RGBW);
        const unsigned dmxChannelsPerLed = is4Chan ? 4 : 3;
        const unsigned ledsPerUniverse = is4Chan ? MAX_4_CH_LEDS_PER_UNIVERSE : MAX_3_CH_LEDS_PER_UNIVERSE;
        uint8_t stripBrightness = bri;
        unsigned previousLeds, dmxOffset, ledsTotal;

        if (previousUniverses == 0) {
          if (availDMXLen < 1) return;
          dmxOffset = dataOffset;
          previousLeds = 0;
          // First DMX address is dimmer in DMX_MODE_MULTIPLE_DRGB mode.
          if (DMXMode == DMX_MODE_MULTIPLE_DRGB) {
            stripBrightness = e131_data[dmxOffset++];
            ledsTotal = (availDMXLen - 1) / dmxChannelsPerLed;
          } else {
            ledsTotal = availDMXLen / dmxChannelsPerLed;
          }
        } else {
          // All subsequent universes start at the first channel.
          dmxOffset = (protocol == P_ARTNET) ? 0 : 1;
          const unsigned dimmerOffset = (DMXMode == DMX_MODE_MULTIPLE_DRGB) ? 1 : 0;
          unsigned ledsInFirstUniverse = (((MAX_CHANNELS_PER_UNIVERSE - DMXAddress) + dmxLenOffset) - dimmerOffset) / dmxChannelsPerLed;
          previousLeds = ledsInFirstUniverse + (previousUniverses - 1) * ledsPerUniverse;
          ledsTotal = previousLeds + (dmxChannels / dmxChannelsPerLed);
        }

        // All LEDs already have values
        if (previousLeds >= totalLen) {
          return;
        }



        // EMZ not doing the realtime lock anymore, just freezing the segments we want to update with artnet data
        // realtimeLock(realtimeTimeoutMs, mde);
        // if (realtimeOverride && !(realtimeMode && useMainSegmentOnly)) return;

        if (ledsTotal > totalLen) {
          ledsTotal = totalLen;
        }

        if (DMXMode == DMX_MODE_MULTIPLE_DRGB && previousUniverses == 0) {
          if (bri != stripBrightness) {
            bri = stripBrightness;
            strip.setBrightness(bri, true);
          }
        }

        // MOD1 ////////////////////////////////////////////////////////////
        uint16_t seg_len_1 = strip.getSegment(analog_mod_global[0].segment_id).length();
        float scalar_1 = 0.0f; 
        bool mod1_enable = analog_mod_global[0].osc_artnet_enable | analog_mod_global[0].analog_artnet_enable;

        if(analog_mod_global[0].osc_artnet_enable)
        {
          scalar_1 = analog_mod_global[0].osc_value;
        }
        else if(analog_mod_global[0].analog_artnet_enable)
        {
          scalar_1 = analog_mod_global[0].avg_reading;
        }
        unsigned int stopled_onsegment_1 = (seg_len_1 * scalar_1) ; // segment relative
        // debugI("Mod1: enabled=%u start=%u stop=%u len=%u scalar=%f stopled=%u\r\n", mod1_enable, seg_start_1, seg_stop_1, seg_len_1, scalar_1, stopled_1);


        // MOD2 ////////////////////////////////////////////////////////////
        uint16_t seg_len_2 = strip.getSegment(analog_mod_global[1].segment_id).length();
        float scalar_2 = 0.0f; 
        bool mod2_enable = analog_mod_global[1].osc_artnet_enable | analog_mod_global[1].analog_artnet_enable;

        if(analog_mod_global[1].osc_artnet_enable)
        {
          scalar_2 = analog_mod_global[1].osc_value;
        }
        else if(analog_mod_global[1].analog_artnet_enable)
        {
          scalar_2 = analog_mod_global[1].avg_reading;
        }
        unsigned int stopled_onsegment_2 = (seg_len_2 * scalar_2) ; // segment relative





        // for(uint16_t i=0; i<strip.getSegmentsNum(); i++)
        // {
        //   if(strip.getSegment(0).freeze)
        // }


        // Start at strip 0. 
        uint16_t current_strip = 0;
        int current_led_on_strip = -1;

        // TODO: call find_next_led for all previously set LEDs
        // for(uint16_t i=0; i<previousLeds; i++)
        // {
        //   find_next_led(&current_strip, &current_led_on_strip);
        // }
      
      // bool firstled = find_next_led(&current_strip, &current_led_on_strip);
      // debugI("Got Artnet message, first led result=%u strip=%u led=%u\r\n", firstled, current_strip, current_led_on_strip);
      // firstled = find_next_led(&current_strip, &current_led_on_strip);
      // debugI(" second -  Artnet message, first led result=%u strip=%u led=%u\r\n", firstled, current_strip, current_led_on_strip);



        // EMZ is 4chan RGBW?
        if (!is4Chan) {
          for (unsigned i = previousLeds; i < ledsTotal; i++) 
          {
            uint8_t found = find_next_led(&current_strip, &current_led_on_strip);
            //debugI("success=%u strip=%d led=%d\r\n", found, current_strip, current_led_on_strip);
            if(found)
            {
              // If strip ID matches mod1, and within limit, black out
              // if(mod1_enable && current_strip == analog_mod_global[0].segment_id && current_led_on_strip >= stopled_onsegment_1)
              // {
              //   // blackout the pixel
              //   strip.getSegment(current_strip).setPixelColor(current_led_on_strip, RGBW32(0,0,0,0));
              // }
              // else if(mod2_enable && current_strip == analog_mod_global[1].segment_id && current_led_on_strip >= stopled_onsegment_2)
              // {
              //   // blackout the pixel
              //   strip.getSegment(current_strip).setPixelColor(current_led_on_strip, RGBW32(0,0,0,0));
              // }
              // else
              {
                uint32_t color = RGBW32(e131_data[dmxOffset], e131_data[dmxOffset+1], e131_data[dmxOffset+2],0);
                strip.getSegment(current_strip).setPixelColor(current_led_on_strip, color);
                // strip.getSegment(1).setPixelColor(i, color);
                // debugI("strip.getSegment(%u).setPixelColor(%d, %x)\r\n", current_strip, current_led_on_strip, RGBW32(e131_data[dmxOffset], e131_data[dmxOffset+1], e131_data[dmxOffset+2],0));
              }

            }
            
            
            dmxOffset+=3;
          }
          strip.getSegment(2).beginDraw();
          strip.show();
        }


        // if (useMainSegmentOnly) strip.getMainSegment().beginDraw();
        // if (!is4Chan) {
        //   for (unsigned i = previousLeds; i < ledsTotal; i++) 
        //   {
            
        //     // If we're past the stop pont and we're in the segment we expect
        //     if(mod1_enable && i >= stopled_1 && i >= seg_start_1 && i <= seg_stop_1)
        //     {
        //       // blackout the pixel
        //       //setRealtimePixel(i, 0,0,0, 0);
        //     }
        //     else if(mod2_enable && i>= stopled_2 && i > seg_start_2 && i <= seg_stop_2)
        //     {
        //       //setRealtimePixel(i, 0,0,0, 0);
        //     }
        //     else
        //     {
        //       setRealtimePixel(i, e131_data[dmxOffset], e131_data[dmxOffset+1], e131_data[dmxOffset+2], 0);
        //     }
            
        //     dmxOffset+=3;
        //   }
        // } else {
        //   for (unsigned i = previousLeds; i < ledsTotal; i++) {
        //     if(i < stopled_1)
        //       setRealtimePixel(i, e131_data[dmxOffset], e131_data[dmxOffset+1], e131_data[dmxOffset+2], e131_data[dmxOffset+3]);
        //     else
        //       setRealtimePixel(i, 0,0,0, 0);
        //     dmxOffset+=4;
        //   }
        // }






        break;
      }
    default:
      DEBUG_PRINTLN(F("unknown E1.31 DMX mode"));
      return;  // nothing to do
      break;
  }

  e131NewData = true;
}




static bool find_next_led(uint16_t* current_strip, int* current_led_on_strip)
{
    // debugI("find_next_led: strip=%u led=%d\r\n", *current_strip, *current_led_on_strip);
    // debugI(" -- strip %u/%u len=%u\r\n", *current_strip, strip.getSegmentsNum(), strip.getSegment(*current_strip).length());

    // Check if next LED on the current strip is available; if so return it
    if((strip.getSegment(*current_strip).freeze == true) && ((*current_led_on_strip+1) < strip.getSegment(*current_strip).length()))
    {
      *current_led_on_strip = *current_led_on_strip + 1;
      return true;
    }

    // If next is after the bounds, then we need to find the next strip that is frozen
    // Loop until next frozen strip is found


    while(strip.getSegmentsNum() > 0 && *current_strip < strip.getSegmentsNum()-1)
    {
      *current_strip = *current_strip + 1;

      // debugI("checking strip %U \r\n", *current_strip);
      if(strip.getSegment(*current_strip).freeze)
      {
        // debugI("Strip OK\r\n");
        // OK, found a segment we can use; reset the pixel count
        *current_led_on_strip = 0;
        return true;
      }
    }

    // debugI("No more strips\r\n");
    // Didn't find anything, no LEDs available
    return false;
}




void handleArtnetPollReplyEMZ(IPAddress ipAddress) {
  ArtPollReply artnetPollReply;
  prepareArtnetPollReply(&artnetPollReply); // use std func 

  unsigned startUniverse = e131Universe;
  unsigned endUniverse = e131Universe;

  switch (DMXMode) {
    case DMX_MODE_DISABLED:
      break;

    case DMX_MODE_SINGLE_RGB:
    case DMX_MODE_SINGLE_DRGB:
    case DMX_MODE_PRESET:
    case DMX_MODE_EFFECT:
    case DMX_MODE_EFFECT_W:
    case DMX_MODE_EFFECT_SEGMENT:
    case DMX_MODE_EFFECT_SEGMENT_W:
      break;  // 1 universe is enough

    case DMX_MODE_MULTIPLE_DRGB:
    case DMX_MODE_MULTIPLE_RGB:
    case DMX_MODE_MULTIPLE_RGBW:
      {
        bool is4Chan = (DMXMode == DMX_MODE_MULTIPLE_RGBW);
        const unsigned dmxChannelsPerLed = is4Chan ? 4 : 3;
        const unsigned dimmerOffset = (DMXMode == DMX_MODE_MULTIPLE_DRGB) ? 1 : 0;
        const unsigned dmxLenOffset = (DMXAddress == 0) ? 0 : 1; // For legacy DMX start address 0
        const unsigned ledsInFirstUniverse = (((MAX_CHANNELS_PER_UNIVERSE - DMXAddress) + dmxLenOffset) - dimmerOffset) / dmxChannelsPerLed;
        const unsigned totalLen = strip.getLengthTotal();

        if (totalLen > ledsInFirstUniverse) {
          const unsigned ledsPerUniverse = is4Chan ? MAX_4_CH_LEDS_PER_UNIVERSE : MAX_3_CH_LEDS_PER_UNIVERSE;
          const unsigned remainLED = totalLen - ledsInFirstUniverse;

          endUniverse += (remainLED / ledsPerUniverse);

          if ((remainLED % ledsPerUniverse) > 0) {
            endUniverse++;
          }

          if ((endUniverse - startUniverse) > E131_MAX_UNIVERSE_COUNT) {
            endUniverse = startUniverse + E131_MAX_UNIVERSE_COUNT - 1;
          }
        }
        break;
      }
    default:
      DEBUG_PRINTLN(F("unknown E1.31 DMX mode"));
      return;  // nothing to do
      break;
  }

  if (DMXMode != DMX_MODE_DISABLED) {
    for (unsigned i = startUniverse; i <= endUniverse; ++i) {
      sendArtnetPollReplyEMZ(&artnetPollReply, ipAddress, i);
    }
  }
}


void sendArtnetPollReplyEMZ(ArtPollReply *reply, IPAddress ipAddress, uint16_t portAddress) {
  reply->reply_net_sw = (uint8_t)((portAddress >> 8) & 0x007F);
  reply->reply_sub_sw = (uint8_t)((portAddress >> 4) & 0x000F);
  reply->reply_sw_out[0] = (uint8_t)(portAddress & 0x000F);

  snprintf_P((char *)reply->reply_node_report, sizeof(reply->reply_node_report)-1, PSTR("#0001 [%04u] OK - WLED v" TOSTRING(WLED_VERSION)), pollReplyCount);

  if (pollReplyCount < 9999) {
    pollReplyCount++;
  } else {
    pollReplyCount = 0;
  }

  notifierUdp.beginPacket(ipAddress, PROTOFUSION_ARTNET_PORT);
  notifierUdp.write(reply->raw, sizeof(ArtPollReply));
  notifierUdp.endPacket();

  reply->reply_bind_index++;
}
