#include "Arduino.h"
#include "Wire.h"
#include <matrix.h>

#define ACC_SAD 0x19
#define ACC_SUB_SPEED 0x20
#define ACC_SUB_OUT (0x28|0x80) // initial register + incrementing output

#define MAG_SAD 0x1E
#define MAG_SUB_SPEED 0x00
#define MAG_SUB_GAIN 0x01
#define MAG_SUB_MODE 0x02
#define MAG_SUB_OUT (0x03|0x80) // initial register + incrementing output

#define GYR_SAD 0x6B
#define GYR_SUB_ENABLE 0x20
#define GYR_SUB_OUT (0x28|0x80) // initial register + incrementing output

#define LED_PIN 13
#define SERIAL_BAUD 9600

void magInit(void)
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

void accInit(void)
{	
	Wire.beginTransmission(ACC_SAD); // slave-address
	Wire.write(ACC_SUB_SPEED); // sub-address
	Wire.write(0x57); // 100hz, all enable, normal power
	Wire.endTransmission(1);
}

void gyrInit(void)
{
	Wire.beginTransmission(GYR_SAD); // slave-address
	Wire.write(GYR_SUB_ENABLE); // sub-address
	Wire.write(0x3F); // 100hz, all enable, turn on
	Wire.endTransmission(1);
}

void setup()
{
	pinMode(LED_PIN, OUTPUT);     // set pin as output
	Serial.begin(SERIAL_BAUD);
	Wire.begin();
	accInit();
	magInit();
	gyrInit();	
}

void gyrUpdate(struct vec3 * vec)
{
	Wire.beginTransmission(GYR_SAD);
	Wire.write(GYR_SUB_OUT);
	Wire.endTransmission(0);
	Wire.requestFrom(GYR_SAD, 6);
	unsigned int XL = Wire.read();
	unsigned int XH = Wire.read();
	unsigned int YL = Wire.read();
	unsigned int YH = Wire.read();
	unsigned int ZL = Wire.read();
	unsigned int ZH = Wire.read();	
	
	vec->data[0] = (short)(XH << 8 | XL);
	vec->data[1] = (short)(YH << 8 | YL);
	vec->data[2] = (short)(ZH << 8 | ZL);
}

void accUpdate(struct vec3 * vec)
{
	Wire.beginTransmission(ACC_SAD);
	Wire.write(ACC_SUB_OUT);
	Wire.endTransmission(0);
	Wire.requestFrom(ACC_SAD, 6);
	unsigned int XL = Wire.read();
	unsigned int XH = Wire.read();
	unsigned int YL = Wire.read();
	unsigned int YH = Wire.read();
	unsigned int ZL = Wire.read();
	unsigned int ZH = Wire.read();	
	
	vec->data[0] = (short)(XH << 8 | XL);
	vec->data[1] = (short)(YH << 8 | YL);
	vec->data[2] = (short)(ZH << 8 | ZL);
}	

void magUpdate(struct vec3 * vec)
{
Wire.beginTransmission(MAG_SAD);
	Wire.write(MAG_SUB_OUT);
	Wire.endTransmission(0);
	Wire.requestFrom(MAG_SAD, 6);
	unsigned int XH = Wire.read();
	unsigned int XL = Wire.read();
	unsigned int ZH = Wire.read();
	unsigned int ZL = Wire.read();
	unsigned int YH = Wire.read();
	unsigned int YL = Wire.read();	
	
	vec->data[0] = (short)(XH << 8 | XL);
	vec->data[1] = (short)(YH << 8 | YL);
	vec->data[2] = (short)(ZH << 8 | ZL);
}	

void loop()
{	
	//static mat3 matOrientation 
	
	struct vec3 vecGyrData;
	struct vec3 vecMagData;
	struct vec3 vecAccData;
	
	gyrUpdate(&vecGyrData);
	//magUpdate(&vecMagData);
	//accUpdate(&vecAccData);
	
	
	
	
	/*
	vec3Print(&vecAccData); 
	Serial.printf(" ");
	vec3Print(&vecMagData); 
	Serial.printf(" ");
	vec3Print(&vecGyrData); 
	Serial.printf("\n\r");
	*/
	///Serial.printf("%f %f %f\n\r", accX, accY, accZ);
	delay(100);
	
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