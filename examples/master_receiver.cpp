

#include <TinyWire.h>

#define led_pin 1
#define error_led_pin 4
#define button_pin 3

byte slave_address = 10;


void setup() {
	// config led_pin as Output for driving an LED
	pinMode(led_pin, OUTPUT);
	// config error_led_pin as Output for driving an LED
	pinMode(error_led_pin, OUTPUT);
	// config button_pin als INPUT for a connected button (normally open; connects to GND)
	pinMode( button_pin, INPUT_PULLUP);
	
	// config TinyWire library for I2C master functionality
	TinyWire.begin();
}

void loop() {
	// check, if the button was pressed
	if(digitalRead( button_pin ) == 0) {
		// request 1 byte from the slave with address slave_address
		// returns 0, if there was no error. (otherwise you can find the different error code definitions in TinyWire.h)
		if( TinyWire.requestFrom( slave_address, 1 ) == 0 ) {
			// read all bytes in buffer
			while(TinyWire.available()) {
				// if the read byte is an 'a', toggle the LED
				if(TinyWire.read()=='a') digitalWrite(led_pin,!digitalRead(led_pin));
			}
		} else {
			// if there was an error, turn on the error LED
			digitalWrite( error_led_pin, HIGH);
		}
		delay(300);
		// reset the error led after a short delay
		digitalWrite( error_led_pin, LOW);
	}
}
