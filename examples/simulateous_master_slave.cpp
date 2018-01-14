

#include <TinyWire.h>

#define led_pin 1
#define button_pin 3
#define error_led_pin 4

byte own_address = 10;
byte slave_address = 11;


void setup() {
	// config led_pin as Output for driving an LED
	pinMode(led_pin, OUTPUT);
	// config error_led_pin as Output for driving an LED
	pinMode(error_led_pin, OUTPUT);
	// config button_pin als INPUT for a connected button (normally open; connects to GND)
	pinMode( button_pin, INPUT_PULLUP);
	
	// config TinyWire library for I2C slave functionality
	TinyWire.begin( own_address );
	// set slave receive callback
	TinyWire.onReceive( onI2CReceive );
}

void loop() {
	// check, if the button was pressed
	if(digitalRead(button_pin) == 0) {
		// begin a master transmission with the specified slave address
		// the library temporarily takes the functionality of a master during sending
		TinyWire.beginTransmission( slave_address );
		// fill the send buffer
		TinyWire.send('b');
		// execute the master sending and check for an error
		// returns 0 if there was no error (otherwise you can find the different error code definitions in TinyWire.h)
		if(TinyWire.endTransmission()!=0) {
			// turn on the error LED, if there was an error
			digitalWrite(error_led_pin, HIGH);
		}
		delay(300);
		// reset error LED after a short delay
		digitalWrite(error_led_pin, LOW);
	}
}

void onI2CReceive(int howMany){
	// loops, until all received bytes are read
	while(TinyWire.available()>0){
		// toggles the led everytime, when an 'a' is received
		if(TinyWire.read()=='a') digitalWrite(led_pin, !digitalRead(led_pin);
	}
}
