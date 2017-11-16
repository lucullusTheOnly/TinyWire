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
 * The user function callbacks for slave receiving and request are called from ISRs, so they shouldn't be to long. Also I'm not sure, if it is save to use master functions inside of it.
 * The library now watches the bus and tracks, if it is busy. With this it only start a master send operation, if the bus is free. Otherwise it returns the corresponding error
 * For detecting Stop Conditions the library uses the PinChangeInterrupt on SDA, so you have to change the library, if you want to use it. (There is a Stop Condition flag in the USISR register, but it isn't connected to an interrupt and it is not certain, when this bit is being triggered. So I used this workaround.)
 * The library now checks the Data Output Collision Bit in the USISR register to provide simple bus arbitration. I'm not sure, if this is sufficient, but I don't know an other way to implement bus arbitration without using an extra pin to check the SDA lines state.
 * using the library as a pure master can block the bus for other masters. I didn't find a solution for this. You can either initialize the library as slave and use the simultaneous master/slave functionality or you can disable the the library with the end() function
