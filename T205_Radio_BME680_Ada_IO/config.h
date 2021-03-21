#ifndef __CONFIG__
#define __CONFIG__

//#define  VILLA_ASTRID 1
#define  LILLA_ASTRID 1
//#define  SIIRTOLA 1
#include "secrets.h"
#include "AdafruitIO_WiFi.h"

#if defined (__AVR_ATmega328P__)  // Feather 328P w/wing
  #define RFM69_INT     3  // 
  #define RFM69_CS      4  //
  #define RFM69_RST     2  // "A"
  #define LED           13
#endif

#if defined (Astrid_ATmega328P)  // made  by Tom Höglund
  #define RFM69_INT     2  // 
  #define RFM69_CS      10  //
  #define RFM69_RST     9  // 
  #define LED           13
#endif

#if defined(ESP8266)    // ESP8266 feather w/wing
  #define RFM69_CS      2    // "E"
  #define RFM69_RST     16   // "D"
  #define RFM69_INT     15   // "B"
  #define RFM69_IRQN    digitalPinToInterrupt(RFM69_IRQ )
  #define LED           0
#endif

#if defined(ESP32)    // ESP32 feather w/wing
  #define RFM69_RST     32   // D
  #define RFM69_CS      14   // E
  #define RFM69_INT     33   // B
  #define RFM69_IRQN    digitalPinToInterrupt(RFM69_INT )
  #define LED           13
#endif

#if defined(XX_ESP32)    // ESP32 feather w/wing
  #define RFM69_RST     13   // same as LED
  #define RFM69_CS      33   // "B"
  #define RFM69_INT     27   // "A"
  #define LED           13
#endif

AdafruitIO_WiFi io(IO_USERNAME, IO_KEY, WIFI_SSID, WIFI_PASS);

#endif  //__CONFIG__
