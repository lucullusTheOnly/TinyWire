

#include <TinyWire.h>

#define led_pin 1

byte own_address = 10;


void setup() {
	// config led_pin as Output for driving an LED
	pinMode(led_pin, OUTPUT);
	
	// config TinyWire library for I2C slave functionality
	TinyWire.begin( own_address );
}

void loop() {
	// loops, until all received bytes are read
	while(TinyWire.available()>0){
		// toggles the led everytime, when an 'a' is received
		if(TinyWire.read()=='a') digitalWrite(led_pin, !digitalRead(led_pin);
	}
}
