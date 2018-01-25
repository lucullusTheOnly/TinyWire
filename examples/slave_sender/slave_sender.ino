#include <TinyWire.h>

byte own_address = 10;


void setup() {
	// config TinyWire library for I2C slave functionality
	TinyWire.begin( own_address );
	// register a handler function in case of a request from a master
	TinyWire.onRequest( onI2CRequest );
}

void loop() {

}

// Request Event handler function
//  --> Keep in mind, that this is executed in an interrupt service routine. It shouldn't take long to execute
void onI2CRequest() {
	// sends one byte with content 'b' to the master, regardless how many bytes he expects
	// if the buffer is empty, but the master is still requesting, the slave aborts the communication
	// (so it is not blocking)
	TinyWire.send('b');
}
