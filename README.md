# TinyWire
Composite Master and Slave I2C library for Atmels ATTiny microcontrollers
Class for the new composed TinyWire library for I2C communication on ATTiny's
The complete library is based on the librarys TinyWireM and TinyWireS, which were originally posted by
BroHogan and then modified by jkl. I recomposed their code to unite slave and master functionality similar
to the behavior of the Wire library on an Arduino. The TinyWireM library is available on https://github.com/adafruit/TinyWireM
and the TinyWireS library on https://github.com/rambo/TinyWire/tree/master/TinyWireS
I hope I've given all people the right credit for their work.

## Notes
 * It is very important to use pullup resistors for both lines (e.g. 2 kOhm)
 * The slave part receives also at the general call address 0 (request for sending to master with this address are not possible in this protocol)
 * The user function callbacks for slave receiving and request are called from ISRs, so they shouldn't be to long. Also I'm not sure, if it is save to use master functions inside of it.
 * The library now watches the bus and tracks, if it is busy. With this it only starts a master send operation, if the bus is free. Otherwise it returns the corresponding error
 * For detecting Stop Conditions the library uses the PinChangeInterrupt on SDA, so you have to change the library, if you want to use it yourself. (There is a Stop Condition flag in the USISR register, but it isn't connected to an interrupt and it is not certain, when this bit is being triggered. So I used this workaround.)
 * The library now checks the Data Output Collision Bit in the USISR register on every clock cycle of a master operation (but of course not, when an (N)ACK is received) to provide Bus Arbitration. When the bit is set (this happens, when bit 7 of the USI data shift register differs from the state of the SDA pin) the corresponding error is returned and the library falls back to slave mode. Currently it doesn't check, if another master wants to address it (this has to be implemented in future).
 * using the library as a pure master can block the bus for other masters. I didn't find a solution for this. You can either initialize the library as slave and use the simultaneous master/slave functionality or you can disable the the library with the end() function

## TODO
 * Add PinChange Interrupt Vector, Enable bit and Flag bit in device dependent definitions in twi.h for all other devices than ATTiny25/45/85
 * After an Arbitration Lost, the library has to check, if it was addressed by another master. If so, it has to receive the masters message.
 * Check compatibility with other clock speeds than 8MHz
