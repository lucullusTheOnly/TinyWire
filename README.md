# TinyWire
Composite Master and Slave I2C library for Atmels ATTiny microcontrollers

Wrapperclass for the new composed TinyWire library for I2C communication on ATTiny's
The complete library is based on the librarys TinyWireM and TinyWireS, which were originally posted by
BroHogan and then modified by jkl. I recomposed their code to unite slave and master functionality similar
to the behavior of the Wire library on an Arduino. The TinyWireM library is available on https://github.com/adafruit/TinyWireM
and the TinyWireS library on https://github.com/rambo/TinyWire/tree/master/TinyWireS
I hope I've given all people the right credit for their work.

## Notes
 * It is very important to use pullup resistors for both lines
 * The user function callback in the event of a slave receiving data is not working currently. The receiving is only stopped by a master sending a stop or restart condition, but detecting this in the ISR doesn't work properly and I don't know, why
 * using the library as a pure master can block the bus for other masters. I didn't find a solution for this. You can either initialize the library as slave and use the simultaneous master/slave functionality or you can disable the the library with the end() function
