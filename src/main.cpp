#include "Arduino.h"
#include "Wire.h"

#define ACC_SAD 0x19
#define ACC_SUB_SPEED 0x20
#define ACC_SUB_OUT (0x28|0x80) // initial register + incrementing output

#define MAG_SAD 0x1E
#define MAG_SUB_SPEED 0x00
#define MAG_SUB_GAIN 0x01
#define MAG_SUB_MODE 0x02
#define MAG_SUB_OUT (0x03|0x80) // initial register + incrementing output

#define LED_PIN 13
#define SERIAL_BAUD 9600

void mag_init(void)
{	
	Wire.beginTransmission(MAG_SAD); // slave-address
	Wire.write(MAG_SUB_GAIN);
	Wire.write(0x20); // gain is +/- 1.3 Gauss
	Wire.endTransmission(1);
	
	Wire.beginTransmission(MAG_SAD); // slave-address
	Wire.write(MAG_SUB_SPEED);
	Wire.write(0x14); // speed is 30 hz
	Wire.endTransmission(1);
	
	Wire.beginTransmission(MAG_SAD); // slave-address
	Wire.write(MAG_SUB_MODE);
	Wire.write(0x00); // mode is continuous
	Wire.endTransmission(1);
}

void acc_init(void)
{	
	Wire.beginTransmission(ACC_SAD); // slave-address
	Wire.write(ACC_SUB_SPEED); // sub-address
	Wire.write(0x57); // 100hz, all enable, normal power
	Wire.endTransmission(1);
}

void setup()
{
	pinMode(LED_PIN, OUTPUT);     // set pin as output
	Serial.begin(SERIAL_BAUD);
	delay(2000);
	Serial.printf("After delay\n\r");
	Wire.begin();
	acc_init();
	mag_init();
	
	Serial.printf("Initial connect\n\r");

	Serial.printf("After config\n\r");
	
}

void loop()
{	
	Wire.beginTransmission(ACC_SAD);
	Wire.write(ACC_SUB_OUT);
	Wire.endTransmission(0);
	Wire.requestFrom(ACC_SAD, 6);
	unsigned int XLA = Wire.read();
	unsigned int XHA = Wire.read();
	unsigned int YLA = Wire.read();
	unsigned int YHA = Wire.read();
	unsigned int ZLA = Wire.read();
	unsigned int ZHA = Wire.read();	
	
	float accX = (short)(XHA << 8 | XLA);
	float accY = (short)(YHA << 8 | YLA);
	float accZ = (short)(ZHA << 8 | ZLA);
	
	
	Wire.beginTransmission(MAG_SAD);
	Wire.write(MAG_SUB_OUT);
	Wire.endTransmission(0);
	Wire.requestFrom(MAG_SAD, 6);
	
	unsigned int XHM = Wire.read();
	unsigned int XLM = Wire.read();
	unsigned int ZHM = Wire.read();
	unsigned int ZLM = Wire.read();
	unsigned int YHM = Wire.read();
	unsigned int YLM = Wire.read();	
	
	float magX = (short)(XHM << 8 | XLM);
	float magY = (short)(YHM << 8 | YLM);
	float magZ = (short)(ZHM << 8 | ZLM);
	
	Serial.printf("%f %f %f %f %f %f\n\r", accX, accY, accZ, magX, magY, magZ);
	
	
	///Serial.printf("%f %f %f\n\r", accX, accY, accZ);
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