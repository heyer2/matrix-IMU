#include "Arduino.h"
#include "Wire.h"
#include <matrix.h>

#define int16 short

#define ACC_SAD 0x19
#define ACC_SUB_SPEED 0x20
#define ACC_SUB_OUT (0x28|0x80) // initial register + incrementing output

#define MAG_SAD 0x1E
#define MAG_SUB_SPEED 0x00
#define MAG_SUB_GAIN 0x01
#define MAG_SUB_MODE 0x02
#define MAG_SUB_OUT (0x03|0x80) // initial register + incrementing output

#define MAG_BIAS_X (-4.17705880568963)
#define MAG_BIAS_Y (-48.5696006492905)
#define MAG_BIAS_Z 398.835722468053

#define MAG_GAIN_X 0.00234202173981799
#define MAG_GAIN_Y 0.00241234250943554
#define MAG_GAIN_Z 0.00277519214826859

#define GYR_SAD 0x6B

#define GYR_SUB_CTRL1 				  0x20
#define GYR_SUB_CTRL1_MASK_ODR_100HZ  0x00
#define GYR_SUB_CTRL1_MASK_ODR_200HZ  0x40
#define GYR_SUB_CTRL1_MASK_ODR_400HZ  0x80
#define GYR_SUB_CTRL1_MASK_ODR_800HZ  0xC0
#define GYR_SUB_CTRL1_MASK_BW_LOWEST  0x00
#define GYR_SUB_CTRL1_MASK_BW_LOW 	  0x01
#define GYR_SUB_CTRL1_MASK_BW_HIGH 	  0x02
#define GYR_SUB_CTRL1_MASK_BW_HIGHEST 0x03
#define GYR_SUB_CTRL1_MASK_ENABLE 	  0x0F

#define GYR_SUB_CTRL4 				     0x23
#define GYR_SUB_CTRL4_MASK_SCALE_245DPS  0x00
#define GYR_SUB_CTRL4_MASK_SCALE_500DPS  0x10
#define GYR_SUB_CTRL4_MASK_SCALE_2000DPS 0x20

#define GYR_SUB_OUT 0x28 // Initial register, use incrementing read

#define GYR_SUB_STATUS 			   0x27
#define GYR_SUB_STATUS_MASK_NEWSET 0x04

#define GYR_LSB_RAD_245DPS  0.0001527163095
#define GYR_LSB_RAD_500DPS  0.0003054326190
#define GYR_LSB_RAD_2000DPS 0.0012217304764

#define GYR_BIAS_SAMPLES 1000

#define SERIAL_BAUD 115200

#define ORTHO_FIX_INTERVAL 10000

#define MILLIS2SEC 0.001;

enum outputRate {HZ_100, HZ_200, HZ_400, HZ_800};
enum bandWidth {BW_LOWEST, BW_LOW, BW_HIGH, BW_HIGHEST};
enum fullScale {DPS_245, DPS_500, DPS_2000};

struct gyro {
	int flagNewAvail;
	int timeUsed;
	struct vec3 vecVel;
	struct vec3 vecBias;
	enum outputRate ODR;
	enum bandWidth BW;
	enum fullScale FS;
};

void I2CWriteReg(char SAD, char SUB, char byte)
{
	Wire.beginTransmission(SAD);
	Wire.write(SUB);
	Wire.write(byte);
	Wire.endTransmission();
}

char I2CReadReg(char SAD, char SUB)
{	
	Wire.beginTransmission(SAD);
	Wire.write(SUB);
	Wire.endTransmission(0);
	Wire.requestFrom(SAD, 1);
	char test = Wire.read();
	return test;
}

void I2CReadRegSeries(char SAD, char SUB, int bytes) // Will hold the line until all read
{	
	Wire.beginTransmission(SAD);
	Wire.write(SUB | 0x80); // 0x80 activates pointer incrementation
	Wire.endTransmission(1);
	Wire.requestFrom(SAD, bytes);
}

void gyrSetDefault(struct gyro * gyr) 
{
	gyr->ODR = HZ_400;
	gyr->BW  = BW_HIGHEST;
	gyr->FS  = DPS_2000;

	vec3Zero(&gyr->vecBias);
	vec3Zero(&gyr->vecVel);
}

void gyrApply(struct gyro * gyr)
{	
	char byte = 0x00;

	switch(gyr->ODR) {
		case HZ_100: byte |= GYR_SUB_CTRL1_MASK_ODR_100HZ; break;
		case HZ_200: byte |= GYR_SUB_CTRL1_MASK_ODR_200HZ; break;
		case HZ_400: byte |= GYR_SUB_CTRL1_MASK_ODR_400HZ; break;
		case HZ_800: byte |= GYR_SUB_CTRL1_MASK_ODR_800HZ; break;
	}

	switch(gyr->BW) {
		case BW_LOWEST : byte |= GYR_SUB_CTRL1_MASK_BW_LOWEST ; break;
		case BW_LOW    : byte |= GYR_SUB_CTRL1_MASK_BW_LOW    ; break;
		case BW_HIGH   : byte |= GYR_SUB_CTRL1_MASK_BW_HIGH   ; break;
		case BW_HIGHEST: byte |= GYR_SUB_CTRL1_MASK_BW_HIGHEST; break;
	}

	byte |= GYR_SUB_CTRL1_MASK_ENABLE; // This should be made optional later

	I2CWriteReg(GYR_SAD, GYR_SUB_CTRL1, byte);

	byte = 0x00;

	switch(gyr->FS) {
		case DPS_245 : byte |= GYR_SUB_CTRL4_MASK_SCALE_245DPS ; break;
		case DPS_500 : byte |= GYR_SUB_CTRL4_MASK_SCALE_500DPS ; break;
		case DPS_2000: byte |= GYR_SUB_CTRL4_MASK_SCALE_2000DPS; break;
	}

	I2CWriteReg(GYR_SAD, GYR_SUB_CTRL4, byte);
}

void gyrGetAvailability(struct gyro * gyr)
{	
	char byte = I2CReadReg(GYR_SAD, GYR_SUB_STATUS);
	gyr->flagNewAvail = (byte & GYR_SUB_STATUS_MASK_NEWSET) > 0;
}

void gyrUpdate(struct gyro * gyr)
{	
	I2CReadRegSeries(GYR_SAD, GYR_SUB_OUT, 6);
	unsigned int XL = Wire.read();
	unsigned int XH = Wire.read();
	unsigned int YL = Wire.read();
	unsigned int YH = Wire.read();
	unsigned int ZL = Wire.read();
	unsigned int ZH = Wire.read();	
	
	gyr->vecVel.data[0] = (int16)(XH << 8 | XL);
	gyr->vecVel.data[1] = (int16)(YH << 8 | YL);
	gyr->vecVel.data[2] = (int16)(ZH << 8 | ZL);
	
	switch(gyr->FS) {
		case DPS_245 : vec3Mult(&gyr->vecVel, GYR_LSB_RAD_245DPS) ; break;
		case DPS_500 : vec3Mult(&gyr->vecVel, GYR_LSB_RAD_500DPS) ; break;
		case DPS_2000: vec3Mult(&gyr->vecVel, GYR_LSB_RAD_2000DPS); break;
	}

	vec3Accum(&gyr->vecVel, &gyr->vecBias);

	gyr->flagNewAvail = 0;
}

void gyrGetBias(struct gyro * gyr)
{	
	struct vec3 vecTmp;
	vec3Zero(&vecTmp);
	vec3Zero(&gyr->vecBias);
	
	for (int i = 0; i < GYR_BIAS_SAMPLES; i++) {
		while(!gyr->flagNewAvail)
			gyrGetAvailability(gyr);
		gyrUpdate(gyr);
		vec3Accum(&vecTmp, &gyr->vecVel);
	}
	vec3Mult(&vecTmp, -1.0f / GYR_BIAS_SAMPLES);
	gyr->vecBias = vecTmp;
}

float gyrTimeSinceUse(struct gyro * gyr, int update)
{	
	int millisElapsed = millis() - gyr->timeUsed;
	if (update)
		gyr->timeUsed += millisElapsed;
	return millisElapsed * MILLIS2SEC;
}

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

float accGetDeviation(struct vec3 * vec, float g)
{	
	float length = vec3Length(vec);
	return abs((length - g) / length);
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

	vec->data[0] = (vec->data[0] - MAG_BIAS_X) * MAG_GAIN_X;
	vec->data[1] = (vec->data[1] - MAG_BIAS_Y) * MAG_GAIN_Y;
	vec->data[2] = (vec->data[2] - MAG_BIAS_Z) * MAG_GAIN_Z;
}	

void setup()
{
	Serial.begin(SERIAL_BAUD);
	Wire.begin();
	delay(1500); // For debugging only
}

void loop()
{	
	static struct gyro gyr;
	static struct mat3 matOri;

	static int flagFirstRun = 1;
	if (flagFirstRun) {
		gyrSetDefault(&gyr);
		gyrApply(&gyr);
		gyrGetBias(&gyr);
		mat3Eyes(&matOri);
		flagFirstRun = 0;
	}
	//Serial.printf("All good");
	while(!gyr.flagNewAvail)
		gyrGetAvailability(&gyr);
	//Serial.printf("All good 2");
	gyrUpdate(&gyr);
	//vec3Print(&gyr.vecVel);
	mat3GyrRot(&gyr.vecVel, &matOri, gyrTimeSinceUse(&gyr, 1));
	mat3Print(&matOri);
	Serial.printf("\n\r");
	Serial.send_now();
}