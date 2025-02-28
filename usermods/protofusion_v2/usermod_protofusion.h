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

#define PROTOFUSION_ARTNET_PORT 6454
//from e131
#define MAX_3_CH_LEDS_PER_UNIVERSE 170
#define MAX_4_CH_LEDS_PER_UNIVERSE 128
#define MAX_CHANNELS_PER_UNIVERSE 512


static float lastReading = -1.0f;
static float avg_reading = 0.0f;


void handleArtnetPollReplyEMZ(IPAddress ipAddress);
void sendArtnetPollReplyEMZ(ArtPollReply *reply, IPAddress ipAddress, uint16_t portAddress);
void handleE131PacketEMZ(e131_packet_t* p, IPAddress clientIP, byte protocol);

ESPAsyncE131 secondary_e131(handleE131PacketEMZ);


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
      if(ETH.localIP()[0]) //WLED_CONNECTED)
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












//E1.31 and Art-Net protocol support
void handleE131PacketEMZ(e131_packet_t* p, IPAddress clientIP, byte protocol){
  int uni = 0, dmxChannels = 0;
  uint8_t* e131_data = nullptr;
  int seq = 0, mde = REALTIME_MODE_E131;

  if (protocol == P_ARTNET)
  {
    //DEBUG_PRINTLN(F("Protofusion: got artnet packet."));

    if (p->art_opcode == ARTNET_OPCODE_OPPOLL) {
      //DEBUG_PRINTLN(F(" artnet poll."));
      handleArtnetPollReplyEMZ(clientIP);
      return;
    }
    
    //DEBUG_PRINTLN(F(" artnet realtime."));
    uni = p->art_universe;
    dmxChannels = htons(p->art_length);
    e131_data = p->art_data;
    seq = p->art_sequence_number;
    mde = REALTIME_MODE_ARTNET;
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

        realtimeLock(realtimeTimeoutMs, mde);
        if (realtimeOverride && !(realtimeMode && useMainSegmentOnly)) return;

        if (ledsTotal > totalLen) {
          ledsTotal = totalLen;
        }

        if (DMXMode == DMX_MODE_MULTIPLE_DRGB && previousUniverses == 0) {
          if (bri != stripBrightness) {
            bri = stripBrightness;
            strip.setBrightness(bri, true);
          }
        }

        unsigned int stopled = ledsTotal * (avg_reading / 0.8f);

        if (useMainSegmentOnly) strip.getMainSegment().beginDraw();
        if (!is4Chan) {
          for (unsigned i = previousLeds; i < ledsTotal; i++) {
            if(i < stopled)
              setRealtimePixel(i, e131_data[dmxOffset], e131_data[dmxOffset+1], e131_data[dmxOffset+2], 0);
            else
              setRealtimePixel(i, 0,0,0, 0);
            dmxOffset+=3;
          }
        } else {
          for (unsigned i = previousLeds; i < ledsTotal; i++) {
            if(i < stopled)
              setRealtimePixel(i, e131_data[dmxOffset], e131_data[dmxOffset+1], e131_data[dmxOffset+2], e131_data[dmxOffset+3]);
            else
              setRealtimePixel(i, 0,0,0, 0);
            dmxOffset+=4;
          }
        }
        break;
      }
    default:
      DEBUG_PRINTLN(F("unknown E1.31 DMX mode"));
      return;  // nothing to do
      break;
  }

  e131NewData = true;
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
