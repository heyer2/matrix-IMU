#include "Arduino.h"
#include "Wire.h"


#define LED_PIN 13
#define SERIAL_BAUD 9600

void setup()
{
	pinMode(LED_PIN, OUTPUT);     // set pin as output
	Serial.begin(SERIAL_BAUD);
	delay(2000);
	Serial.printf("After delay\n\r");
	Wire.begin();
	Wire.beginTransmission(0x19); // slave-address
	Serial.printf("Initial connect\n\r");
	Wire.write(0x20); // sub-address
	Wire.write(0x57); // 100hz, all enable, normal power
	Serial.printf("After config\n\r");
	Wire.endTransmission(1);
}

void loop()
{	
	Wire.beginTransmission(0x19);
	Wire.write(0x28);
	Wire.endTransmission();
	Wire.requestFrom(0x19, 1);
	unsigned int XLA = Wire.read();
	
	Wire.beginTransmission(0x19);
	Wire.write(0x29);
	Wire.endTransmission();
	Wire.requestFrom(0x19, 1);
	unsigned int XHA = Wire.read();
	
	float accX = (short)(XHA << 8 | XLA);
	Serial.printf("accX is %f, size of int is %d, size of char is %d\n\r", accX, sizeof(short),sizeof(char));
	delay(200);
	//Serial.printf("Got the first\n\r");
	//unsigned int XHA = Wire.read();
	//unsigned int YLA = Wire.read();
	//unsigned int YHA = Wire.read();
	//unsigned int ZLA = Wire.read();
	//unsigned int ZHA = Wire.read();
	//Serial.printf("Received stuff\n\r");
	//float accX = (int)(XHA | XLA);
	//float accY = (int)(YHA | YLA);
	//float accZ = (int)(ZHA | ZLA);
	
	
	//Serial.printf("%d\n\r", XLA);
	//Serial.printf("%f %f %f\n\r", accX, accY, accZ);
	//Wire.requestFrom(0x32, 6, 1);
	/*
	static float rot = 0;
	while (1) {
		Serial.printf("%f 0 0\n\r", rot);
		rot += PI / 100;
		delay(100);
	}
	*/	
}