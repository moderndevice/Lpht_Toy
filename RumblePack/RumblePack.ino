

/*********************************************************************
  This is an example for our nRF51822 based Bluefruit LE modules
  Uses Adafruit Controller example as a base to control a
  Feather-based board, LED ring, accelerometer and possibly sound
*********************************************************************/

#include <string.h>
#include <Arduino.h>
#include <SPI.h>
#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"
#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
#include <avr/power.h>
#endif

#include "BluefruitConfig.h"

#if SOFTWARE_SERIAL_AVAILABLE
#include <SoftwareSerial.h>
#endif

/*=========================================================================
    APPLICATION SETTINGS

      FACTORYRESET_ENABLE       Perform a factory reset when running this sketch
     
                                Enabling this will put your Bluefruit LE module
                              in a 'known good' state and clear any config
                              data set in previous sketches or projects, so
                                running this at least once is a good idea.
     
                                When deploying your project, however, you will
                              want to disable factory reset by setting this
                              value to 0.  If you are making changes to your
                                Bluefruit LE device via AT commands, and those
                              changes aren't persisting across resets, this
                              is the reason why.  Factory reset will erase
                              the non-volatile memory where config data is
                              stored, setting it back to factory default
                              values.
         
                                Some sketches that require you to bond to a
                              central device (HID mouse, keyboard, etc.)
                              won't work at all with this feature enabled
                              since the factory reset will clear all of the
                              bonding data stored on the chip, meaning the
                              central device won't be able to reconnect.
    MINIMUM_FIRMWARE_VERSION  Minimum firmware version to have some new features
    MODE_LED_BEHAVIOUR        LED activity, valid options are
                              "DISABLE" or "MODE" or "BLEUART" or
                              "HWUART"  or "SPI"  or "MANUAL"
    -----------------------------------------------------------------------*/
#define FACTORYRESET_ENABLE         1
#define MINIMUM_FIRMWARE_VERSION    "0.6.6"
#define MODE_LED_BEHAVIOUR          "MODE"
/*=========================================================================*/

// Create the bluefruit object, either software serial...uncomment these lines
/*
  SoftwareSerial bluefruitSS = SoftwareSerial(BLUEFRUIT_SWUART_TXD_PIN, BLUEFRUIT_SWUART_RXD_PIN);

  Adafruit_BluefruitLE_UART ble(bluefruitSS, BLUEFRUIT_UART_MODE_PIN,
                      BLUEFRUIT_UART_CTS_PIN, BLUEFRUIT_UART_RTS_PIN);
*/

/* ...or hardware serial, which does not need the RTS/CTS pins. Uncomment this line */
// Adafruit_BluefruitLE_UART ble(BLUEFRUIT_HWSERIAL_NAME, BLUEFRUIT_UART_MODE_PIN);

/* ...hardware SPI, using SCK/MOSI/MISO hardware SPI pins and then user selected CS/IRQ/RST */
Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

/* ...software SPI, using SCK/MOSI/MISO user-defined SPI pins and then user selected CS/IRQ/RST */
//Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_SCK, BLUEFRUIT_SPI_MISO,
//                             BLUEFRUIT_SPI_MOSI, BLUEFRUIT_SPI_CS,
//                             BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);


// A small helper
void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}

#define DATA_PIN 9

#define NUM_LEDS 12

#define BRIGHTNESS 50

Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUM_LEDS, DATA_PIN, NEO_GRB + NEO_KHZ800);

// function prototypes over in packetparser.cpp
uint8_t readPacket(Adafruit_BLE *ble, uint16_t timeout);
float parsefloat(uint8_t *buffer);
void printHex(const uint8_t * data, const uint32_t numBytes);

// the packet buffer
extern uint8_t packetbuffer[];
uint8_t colorFlickerGreenOn = 0;  // for colorFlicker effect
uint8_t colorBlinkGreenOn = 0;         // for colorBLinkOn effect
uint32_t  blinkColor, blink2Color, TCblinkColor, TCblink2Color;  // for blink functions
uint16_t  blinkRate, TCblinkRate;
uint32_t totalBlinkTime, TCtotalBlinkTime;
uint8_t  BlinkStart, TCblinkStart, BlinkOn, BlinkOn2, TCBlinkOn;
uint8_t  colorWipeOn1 = 0, colorWipeOn2 = 0, colorWipeOn3 = 0, colorWipeOn4 = 0;


/**************************************************************************/
/*!
    @brief  Sets up the HW an the BLE module (this function is called
            automatically on startup)
*/
/**************************************************************************/
void setup(void)
{

  strip.setBrightness(BRIGHTNESS);
  strip.begin();
  strip.show(); // Initialize all pixels to 'off'


  // while (!Serial);  // required for Flora & Micro
  delay(500);

  Serial.begin(115200);
  Serial.println(F("Adafruit Bluefruit App Controller Example"));
  Serial.println(F("-----------------------------------------"));

  /* Initialise the module */
  Serial.print(F("Initialising the Bluefruit LE module: "));

  if ( !ble.begin(VERBOSE_MODE) )
  {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }
  Serial.println( F("OK!") );

  if ( FACTORYRESET_ENABLE )
  {
    /* Perform a factory reset to make sure everything is in a known state */
    Serial.println(F("Performing a factory reset: "));
    if ( ! ble.factoryReset() ) {
      error(F("Couldn't factory reset"));
    }
  }


  /* Disable command echo from Bluefruit */
  ble.echo(false);

  Serial.println("Requesting Bluefruit info:");
  /* Print Bluefruit information */
  ble.info();

  Serial.println(F("Please use Adafruit Bluefruit LE app to connect in Controller mode"));
  Serial.println(F("Then activate/use the sensors, color picker, game controller, etc!"));
  Serial.println();

  ble.verbose(false);  // debug info is a little annoying after this point!

  /* Wait for connection */
  while (! ble.isConnected()) {
    delay(500);
  }

  Serial.println(F("******************************"));

  // LED Activity command is only supported from 0.6.6
  if ( ble.isVersionAtLeast(MINIMUM_FIRMWARE_VERSION) )
  {
    // Change Mode LED Activity
    Serial.println(F("Change LED activity to " MODE_LED_BEHAVIOUR));
    ble.sendCommandCheckOK("AT+HWModeLED=" MODE_LED_BEHAVIOUR);
  }

  // Set Bluefruit to DATA mode
  Serial.println( F("Switching to DATA mode!") );
  ble.setMode(BLUEFRUIT_MODE_DATA);

  Serial.println(F("******************************"));

}

/**************************************************************************/
/*!
    @brief  Constantly poll for new command or response data
*/
/**************************************************************************/
void loop(void)
{
  colorFlickerGreeen2();
  BlinkColor();
  TwoColorBlink();

  /* Wait for new data to arrive */
  uint8_t len = readPacket(&ble, 25 ); // BLE_READPACKET_TIMEOUT
  if (len == 0) return;  // nothing else will happen till we are connected


  /* Got a packet! */
  // printHex(packetbuffer, len);

  // Color
  if (packetbuffer[1] == 'C') {
    uint8_t red = packetbuffer[2];
    uint8_t green = packetbuffer[3];
    uint8_t blue = packetbuffer[4];
    Serial.print ("RGB #");
    if (red < 0x10) Serial.print("0");
    Serial.print(red, HEX);
    if (green < 0x10) Serial.print("0");
    Serial.print(green, HEX);
    if (blue < 0x10) Serial.print("0");
    Serial.println(blue, HEX);
  }

  // Buttons
  if (packetbuffer[1] == 'B') {
    uint8_t buttnum = packetbuffer[2] - '0';
    boolean pressed = packetbuffer[3] - '0';
    Serial.print ("Button "); Serial.print(buttnum);
    if (pressed) {
      Serial.println(" pressed");
    } else {
      Serial.println(" released");
    }

    doRumblePack(buttnum, pressed);
  }



}

void doRumblePack(uint8_t butnum, boolean pressed) {
  int inputCode = (butnum * 2) + (int)pressed; // encode butt number and pressed into one variable

  Serial.print("inputCode ");
  Serial.println(inputCode);

  switch (inputCode) {

    case 3:   // button1 pushed
      colorWipeOn1 = !colorWipeOn1;
      if (colorWipeOn1) {
        colorWipe(strip.Color(0, 255, 0), 10); // Green
      }
      else {
        colorWipe(strip.Color(0, 0, 0), 0);     // Black
      }
      break;

    case 2:   // button1 released
      break;

    case 5:   // button2 pushed
      colorFlickerGreenOn = !colorFlickerGreenOn;  // green flicker
      if (!colorFlickerGreenOn) {
        colorWipe(strip.Color(0, 0, 0), 0);     // Black
      }
      break;

    case 4:   // button2 released
      // colorFlickerGreenOn = 0;
      //  colorWipe(strip.Color(0, 0, 0), 0);     // Black
      break;

    case 7:   // button3 pushed
      TCBlinkOn = !TCBlinkOn;
      if (TCBlinkOn) {
        // setBlinkColor(uint32_t color, uint32_t color2, uint16_t rate, uint32_t totalBlTime)
        setTwoColorBlink(strip.Color(255, 0, 0), strip.Color(0, 255, 0), 120, 3000); // note total blink time param
      }
      else {
        TCtotalBlinkTime = 0; // turn off blink
        colorWipe(strip.Color(0, 0, 0), 0);     // turn off LEDs if not off
      }
      break;

    case 6:   // button3 released
      break;

    case 9:   // button4 pushed
      BlinkOn2 = !BlinkOn2;
      if (BlinkOn2) {
        // setBlinkColor(uint32_t color, uint32_t color2, uint16_t rate, uint32_t totalBlTime)
        setBlinkColor(strip.Color(255, 0, 0), strip.Color(0, 0, 0), 75, 3000); // note total blink time param
      }
      else {
        totalBlinkTime = 0; // turn off blink
        colorWipe(strip.Color(0, 0, 0), 0);     // turn off LEDs if not off
      }
      break;

    case 8:   // button4 released
      break;

    case 11:   // button5 pushed
      break;

    case 10:   // button5 released
      break;

    case 13:   // button6 pushed
      break;

    case 12:   // button6 released
      break;

    case 15:   // button7 pushed
      break;

    case 14:   // button7 released
      break;

    case 17:   // button7 pushed
      break;

    case 16:   // button7 released
      break;

  }
}


// Fill the dots one after the other with a color
void colorWipe(uint32_t c, uint8_t wait) {
  for (uint16_t i = 0; i < strip.numPixels(); i++) {
    strip.setPixelColor(i, c);
    strip.show();
    delay(wait);
  }
}

void colorFlickerGreeen() {
  if (colorFlickerGreenOn) {
    for (uint16_t i = 0; i < strip.numPixels(); i++) {
      strip.setPixelColor(i, strip.Color(0, random(5) * 64, 0));
    }
    strip.show();
  }
}

void colorFlickerGreeen2() {
  static float offset1, dimmer = 0.02, adder = 1.03;
  if (colorFlickerGreenOn) {
    offset1 += 0.9;
    dimmer *= adder;
    if (dimmer > 1.0) {
      dimmer = 1.0;
      adder =  0.97;
    }
    if (dimmer < 0.03) {
      dimmer = 0.03;
      adder = 1.03;
    }

    for (uint16_t i = 0; i < strip.numPixels(); i++) {
      strip.setPixelColor(i, strip.Color(0, ((sin(offset1 + ((float)i / 2.0)) + 1.0) * 100.0) * dimmer, 0));
    }
    strip.show();
  }
}

void TwoColorBlink() {
  static uint32_t  TClastTime, TCstartTime;
  static uint8_t onOff=0, tidyUp = 0;

  if (TCblinkStart) {
    TCstartTime = millis();
    tidyUp = 1;
    TCblinkStart = 0;  // make sure startTime only gets set once
  }

  if (millis() - TCstartTime < TCtotalBlinkTime) {
    if (millis() - TClastTime > TCblinkRate) {
      onOff = !onOff;
      Serial.print("onOff "); Serial.println(onOff);
      for (uint16_t i = 0; i < strip.numPixels(); i++) {
        if (onOff) {
          if (i % 2 == 1) {
            strip.setPixelColor(i, TCblink2Color);
          }
          else {
            strip.setPixelColor(i, TCblinkColor);
          }
        }
        else {
          if (i % 2 == 1) {
            strip.setPixelColor(i, TCblinkColor);
          }
          else {
            strip.setPixelColor(i, TCblink2Color);
          }
        }
      }
      strip.show();
      TClastTime = millis();
    }  
  }

  else {
    if (tidyUp) { // LEDs might be on - don't leave them on
      for (uint16_t i = 0; i < strip.numPixels(); i++) {
        strip.setPixelColor(i, strip.Color(0, 0, 0));
      }
      strip.show();
      tidyUp = 0;
      TCBlinkOn = 0;
    }
  }
}

void setTwoColorBlink(uint32_t color, uint32_t color2, uint16_t rate, uint32_t totalBlTime) {
  TCblinkColor = color;
  TCblink2Color = color2;
  TCblinkRate = rate;
  TCtotalBlinkTime = totalBlTime;
  TCblinkStart = 1;
}



void BlinkColor() {
  static uint32_t  lastTime, startTime;
  static uint8_t onOff, tidyUp = 0;

  if (BlinkStart) {
    startTime = millis();
    tidyUp = 1;
    BlinkStart = 0;  // make sure startTime only gets set once
  }

  if (millis() - startTime < totalBlinkTime) {
    if (millis() - lastTime > blinkRate) {
      onOff = !onOff;
      for (uint16_t i = 0; i < strip.numPixels(); i++) {
        if (onOff) {
          strip.setPixelColor(i, blinkColor);
        }
        else {
          strip.setPixelColor(i, blink2Color);
        }
      }
      strip.show();
      lastTime = millis();
    }
  }
  else {
    if (tidyUp) { // LEDs might be on - don't leave them on
      for (uint16_t i = 0; i < strip.numPixels(); i++) {
        strip.setPixelColor(i, strip.Color(0, 0, 0));
      }
      strip.show();
      tidyUp = 0;
      BlinkOn2 = 0;
    }
  }
}

void setBlinkColor(uint32_t color, uint32_t color2, uint16_t rate, uint32_t totalBlTime) {
  blinkColor = color;
  blink2Color = color2;
  blinkRate = rate;
  totalBlinkTime = totalBlTime;
  BlinkStart = 1;
}

void allOff() {

}
