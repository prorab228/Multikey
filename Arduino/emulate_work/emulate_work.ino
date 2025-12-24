/*
 *    Example-Code that emulates a DS2401 used as a binary button (like reed-contact - power on, power off)
 *
 *    Tested with
 *    - https://github.com/PaulStoffregen/OneWire on the other side as Master, atmega328@16MHz as Slave
 *    - DS9490R-Master, atmega328@16MHz and teensy3.2@96MHz as Slave
 */

#include "OneWireHub.h"
#include "DS2401.h"  // Serial Number

constexpr uint8_t pin_led       { 13 };
constexpr uint8_t pin_onewire   { D5 };

auto hub     = OneWireHub(pin_onewire);
auto ds1990A = DS2401( 0x01, 0x6A, 0xC6, 0x21, 0xB4, 0x00, 0x00 ); //{ 0x01, 0x6A, 0xC6, 0x21, 0xB4, 0x00, 0x00, 0x52 };

bool blinking(void);


void setup()
{
    hub.attach(ds1990A); // always online
}

void loop()
{
    // following function must be called periodically
    hub.poll();

}
