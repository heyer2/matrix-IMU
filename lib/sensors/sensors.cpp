#include <stdint.h>
#include <sensors.h>
#include <matrix.h>
#include <matrixFix.h>
#include <i2c_t3.h> // If using Teensy 3.x
//#include "Wire.h" // If using arduino

#include "Arduino.h"


#define MILLIS2SEC 0.001;

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

#define GYR_SUB_CTRL4 				  0x23
#define GYR_SUB_CTRL4_MASK_FS_245DPS  0x00
#define GYR_SUB_CTRL4_MASK_FS_500DPS  0x10
#define GYR_SUB_CTRL4_MASK_FS_2000DPS 0x20

#define GYR_SUB_STATUS 			   0x27
#define GYR_SUB_STATUS_MASK_NEWSET 0x08

#define GYR_SUB_OUT 0x28 // Initial register, use incrementing read

#define GYR_LSB_245DPS  0.0001527163095
#define GYR_LSB_500DPS  0.0003054326190
#define GYR_LSB_2000DPS 0.0012217304764

#define GYR_BIAS_SAMPLES_DISCARD 200
#define GYR_BIAS_SAMPLES_USE 1000


#define ACC_SAD 0x19

#define ACC_SUB_CTRL1 				 0x20
#define ACC_SUB_CTRL1_MASK_ODR_1HZ   0x10
#define ACC_SUB_CTRL1_MASK_ODR_10HZ  0x20
#define ACC_SUB_CTRL1_MASK_ODR_25HZ  0x30
#define ACC_SUB_CTRL1_MASK_ODR_50HZ  0x40
#define ACC_SUB_CTRL1_MASK_ODR_100HZ 0x50
#define ACC_SUB_CTRL1_MASK_ODR_200HZ 0x60
#define ACC_SUB_CTRL1_MASK_ODR_400HZ 0x70
#define ACC_SUB_CTRL1_MASK_ENABLE 	 0x07

#define ACC_SUB_CTRL4			  0x23
#define ACC_SUB_CTRL4_MASK_FS_2G  0x00
#define ACC_SUB_CTRL4_MASK_FS_4G  0x10
#define ACC_SUB_CTRL4_MASK_FS_8G  0x20
#define ACC_SUB_CTRL4_MASK_FS_16G 0x30
#define ACC_SUB_CTRL4_HIGHRES     0x08 // If disabled, accelerometer data is stored in the upper 12 bits

#define ACC_SUB_STATUS 			   0x27
#define ACC_SUB_STATUS_MASK_NEWSET 0x08

#define ACC_SUB_OUT 0x28 // Initial register, use incrementing read

#define ACC_LSB_2G  0.001
#define ACC_LSB_4G  0.002
#define ACC_LSB_8G  0.004
#define ACC_LSB_16G 0.012


#define MAG_SAD 0x1E

#define MAG_SUB_CRA 				0x00
#define MAG_SUB_CRA_MASK_ODR_0x75HZ 0x00
#define MAG_SUB_CRA_MASK_ODR_1x50HZ 0x04
#define MAG_SUB_CRA_MASK_ODR_3x00HZ 0x08
#define MAG_SUB_CRA_MASK_ODR_7x50HZ 0x0C
#define MAG_SUB_CRA_MASK_ODR_15HZ   0x10
#define MAG_SUB_CRA_MASK_ODR_30HZ  	0x14
#define MAG_SUB_CRA_MASK_ODR_75HZ	0x18
#define MAG_SUB_CRA_MASK_ODR_220HZ	0x1C  // More like 250/333, possibly related to magnetomer ready hack

#define MAG_SUB_CRB 				 0x01
#define MAG_SUB_CRB_MASK_FS_1x3GAUSS 0x20
#define MAG_SUB_CRB_MASK_FS_1x9GAUSS 0x40
#define MAG_SUB_CRB_MASK_FS_2x5GAUSS 0x60
#define MAG_SUB_CRB_MASK_FS_4x0GAUSS 0x80
#define MAG_SUB_CRB_MASK_FS_4x7GAUSS 0xA0
#define MAG_SUB_CRB_MASK_FS_5x6GAUSS 0xC0
#define MAG_SUB_CRB_MASK_FS_8x1GAUSS 0xF0

#define MAG_SUB_MR 			   0x02
#define MAG_SUB_MR_MASK_ENABLE 0x00 // Continuous conversion mode, only useful one

#define MAG_SUB_SR 			     0x09 // This register is broken it seems, usage differs from the data-sheet
#define MAG_SUB_SR_MASK_RELEVANT 0x03

#define MAG_SUB_OUT 0x03

#define MAG_LSB_1x3GAUSS_XY (1.0/1100)
#define MAG_LSB_1x3GAUSS_Z  (1.0/980)
#define MAG_LSB_1x9GAUSS_XY (1.0/855)
#define MAG_LSB_1x9GAUSS_Z  (1.0/760)
#define MAG_LSB_2x5GAUSS_XY (1.0/670)
#define MAG_LSB_2x5GAUSS_Z  (1.0/600)
#define MAG_LSB_4x0GAUSS_XY (1.0/450)
#define MAG_LSB_4x0GAUSS_Z  (1.0/400)
#define MAG_LSB_4x7GAUSS_XY (1.0/400)
#define MAG_LSB_4x7GAUSS_Z  (1.0/355)
#define MAG_LSB_5x6GAUSS_XY (1.0/330)
#define MAG_LSB_5x6GAUSS_Z  (1.0/295)
#define MAG_LSB_8x1GAUSS_XY (1.0/230)
#define MAG_LSB_8x1GAUSS_Z  (1.0/205)

#define MAG_GAIN_X float2Fix(0.01326922)//(-4.1770588056896)
#define MAG_GAIN_Y float2Fix(0.01278126)//(-48.569600649290)
#define MAG_GAIN_Z float2Fix(0.01115709)//(398.835722468053)

#define MAG_BIAS_X float2Fix(0.00032732)
#define MAG_BIAS_Y float2Fix(0.001090738056514)
#define MAG_BIAS_Z float2Fix(0.012563747364052)


static void I2CWriteReg(char SAD, char SUB, char byte)
{
	Wire.beginTransmission(SAD);
	Wire.write(SUB);
	Wire.write(byte);
	Wire.endTransmission();
}

static char I2CReadReg(char SAD, char SUB)
{	
	Wire.beginTransmission(SAD);
	Wire.write(SUB);
	Wire.endTransmission(0);
	Wire.requestFrom(SAD, 1);
	return Wire.read();
}

static void I2CReadRegSeries(char SAD, char SUB, int bytes) // Will hold the line until all read
{	
	Wire.beginTransmission(SAD);
	Wire.write(SUB | 0x80); // 0x80 activates pointer incrementation
	Wire.endTransmission(1);
	Wire.requestFrom(SAD, bytes);
}

void gyrSetDefault(struct gyro * gyr) 
{
	gyr->ODR = g_HZ_800;
	gyr->BW  = BW_HIGHEST;
	gyr->FS  = DPS_2000;

	vec3fZero(&gyr->vecBias);
	vec3fZero(&gyr->vecGyr);
	gyr->timeUsed = 0;
	gyr->flagNewAvail = 0;
}

void gyrApply(struct gyro * gyr)
{	
	char byte = 0x00;
	switch(gyr->ODR) {
		case g_HZ_100: byte |= GYR_SUB_CTRL1_MASK_ODR_100HZ; break;
		case g_HZ_200: byte |= GYR_SUB_CTRL1_MASK_ODR_200HZ; break;
		case g_HZ_400: byte |= GYR_SUB_CTRL1_MASK_ODR_400HZ; break;
		case g_HZ_800: byte |= GYR_SUB_CTRL1_MASK_ODR_800HZ; break;
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
		case DPS_245 : byte |= GYR_SUB_CTRL4_MASK_FS_245DPS ; break;
		case DPS_500 : byte |= GYR_SUB_CTRL4_MASK_FS_500DPS ; break;
		case DPS_2000: byte |= GYR_SUB_CTRL4_MASK_FS_2000DPS; break;
	}
	I2CWriteReg(GYR_SAD, GYR_SUB_CTRL4, byte);
}

void gyrGetAvailability(struct gyro * gyr)
{	
	char byte = I2CReadReg(GYR_SAD, GYR_SUB_STATUS);
	gyr->flagNewAvail = (byte & GYR_SUB_STATUS_MASK_NEWSET) > 0;
}

static void gyrGetRaw(struct gyro * gyr, struct vec3f * vec)
{
	I2CReadRegSeries(GYR_SAD, GYR_SUB_OUT, 6);
	unsigned int XL = Wire.read();
	unsigned int XH = Wire.read();
	unsigned int YL = Wire.read();
	unsigned int YH = Wire.read();
	unsigned int ZL = Wire.read();
	unsigned int ZH = Wire.read();	
	
	vec->data[0] = (int16_t)(XH << 8 | XL);
	vec->data[1] = (int16_t)(YH << 8 | YL);
	vec->data[2] = (int16_t)(ZH << 8 | ZL);

	gyr->flagNewAvail = 0;
}

void gyrUpdate(struct gyro * gyr)
{	
	gyrGetRaw(gyr, &gyr->vecGyr);
	vec3fAccum(&gyr->vecGyr, &gyr->vecBias);

	unsigned int microsElapsed = micros() - gyr->timeUsed;
	gyr->timeUsed += microsElapsed;

	// MAX ANGLE IS SUPPOSED TO BE PI not 2PI change this!#!#!#!#!#!#!#
	switch(gyr->FS) {
		case DPS_245: 
			gyr->vecAng.data[0] = (((int64_t)gyr->vecGyr.data[0] << 45) / 360 * 35 / 4000 * microsElapsed / 1000000) >> 16;
			gyr->vecAng.data[1] = (((int64_t)gyr->vecGyr.data[1] << 45) / 360 * 35 / 4000 * microsElapsed / 1000000) >> 16;
			gyr->vecAng.data[2] = (((int64_t)gyr->vecGyr.data[2] << 45) / 360 * 35 / 4000 * microsElapsed / 1000000) >> 16;
			break;
		case DPS_500: 
			gyr->vecAng.data[0] = (((int64_t)gyr->vecGyr.data[0] << 45) / 360 * 35 / 2000 * microsElapsed / 1000000) >> 16;
			gyr->vecAng.data[1] = (((int64_t)gyr->vecGyr.data[1] << 45) / 360 * 35 / 2000 * microsElapsed / 1000000) >> 16;
			gyr->vecAng.data[2] = (((int64_t)gyr->vecGyr.data[2] << 45) / 360 * 35 / 2000 * microsElapsed / 1000000) >> 16;
			break;
		case DPS_2000: 
			gyr->vecAng.data[0] = (((int64_t)gyr->vecGyr.data[0] << 45) / 360 * 35 /  500 * microsElapsed / 1000000) >> 16;
			gyr->vecAng.data[1] = (((int64_t)gyr->vecGyr.data[1] << 45) / 360 * 35 /  500 * microsElapsed / 1000000) >> 16;
			gyr->vecAng.data[2] = (((int64_t)gyr->vecGyr.data[2] << 45) / 360 * 35 /  500 * microsElapsed / 1000000) >> 16;
		break;
	}
}

void gyrGetBias(struct gyro * gyr)
{	
	struct vec3f vecTmp;
	vec3fZero(&gyr->vecBias);
	
	for (int i = 0; i < GYR_BIAS_SAMPLES_DISCARD; i++) {
		while(!gyr->flagNewAvail)
			gyrGetAvailability(gyr);
		gyrGetRaw(gyr, &vecTmp);
		// Serial.printf(" curtmp: %d %d %d ", vecTmp.data[0], vecTmp.data[1], vecTmp.data[2]);
		// Serial.printf(" tr: %d\n\r", i);
		// Serial.send_now();
	}

	for (int i = 0; i < GYR_BIAS_SAMPLES_USE; i++) {
		while(!gyr->flagNewAvail)
			gyrGetAvailability(gyr);
		gyrGetRaw(gyr, &vecTmp);
		vec3fAccum(&gyr->vecBias, &vecTmp);
		// Serial.printf(" curbias: %d %d %d ", gyr->vecBias.data[0], gyr->vecBias.data[1], gyr->vecBias.data[2]);
		// Serial.printf(" curtmp: %d %d %d ", vecTmp.data[0], vecTmp.data[1], vecTmp.data[2]);
		// Serial.printf(" i: %d\n\r", i);
		// Serial.send_now();
	}
	gyr->vecBias.data[0] /= -GYR_BIAS_SAMPLES_USE;
	gyr->vecBias.data[1] /= -GYR_BIAS_SAMPLES_USE;
	gyr->vecBias.data[2] /= -GYR_BIAS_SAMPLES_USE;
	// Serial.printf(" Finalbias: %d %d %d ", gyr->vecBias.data[0], gyr->vecBias.data[1], gyr->vecBias.data[2]);
}

void accSetDefault(struct acce * acc) 
{
	acc->ODR = a_HZ_100; //25
	acc->FS  = G_2;

	vec3fZero(&acc->vecAcc);
	acc->flagNewAvail = 0;
}

void accApply(struct acce * acc)
{	
	char byte = 0x00;
	switch(acc->ODR) {
		case a_HZ_1  : byte |= ACC_SUB_CTRL1_MASK_ODR_1HZ  ; break;
		case a_HZ_10 : byte |= ACC_SUB_CTRL1_MASK_ODR_10HZ ; break;
		case a_HZ_25 : byte |= ACC_SUB_CTRL1_MASK_ODR_25HZ ; break;
		case a_HZ_50 : byte |= ACC_SUB_CTRL1_MASK_ODR_50HZ ; break;
		case a_HZ_100: byte |= ACC_SUB_CTRL1_MASK_ODR_100HZ; break;
		case a_HZ_200: byte |= ACC_SUB_CTRL1_MASK_ODR_200HZ; break;
		case a_HZ_400: byte |= ACC_SUB_CTRL1_MASK_ODR_400HZ; break;
	}
	byte |= ACC_SUB_CTRL1_MASK_ENABLE; // This should be made optional later
	I2CWriteReg(ACC_SAD, ACC_SUB_CTRL1, byte);

	byte = 0x00;
	switch(acc->FS) {
		case G_2 : byte |= ACC_SUB_CTRL4_MASK_FS_2G ; break;
		case G_4 : byte |= ACC_SUB_CTRL4_MASK_FS_4G ; break;
		case G_8 : byte |= ACC_SUB_CTRL4_MASK_FS_8G ; break;
		case G_16: byte |= ACC_SUB_CTRL4_MASK_FS_16G; break;
	}
	byte |= ACC_SUB_CTRL4_HIGHRES;
	I2CWriteReg(ACC_SAD, ACC_SUB_CTRL4, byte);
}

void accGetAvailability(struct acce * acc)
{	
	char byte = I2CReadReg(ACC_SAD, ACC_SUB_STATUS);
	acc->flagNewAvail = (byte & ACC_SUB_STATUS_MASK_NEWSET) > 0;
}

void accUpdate(struct acce * acc)
{	
	// Serial.printf("Time taken: ");
	// int time = micros();
	I2CReadRegSeries(ACC_SAD, ACC_SUB_OUT, 6);
	unsigned int XL = Wire.read();
	unsigned int XH = Wire.read();
	unsigned int YL = Wire.read();
	unsigned int YH = Wire.read();
	unsigned int ZL = Wire.read();
	unsigned int ZH = Wire.read();	
	// Serial.printf("%d \n\r", micros()-time);
	acc->vecAcc.data[0] = (int16_t)(XH << 8 | XL); // The value is stored in the upper 12 bits
	acc->vecAcc.data[1] = (int16_t)(YH << 8 | YL); // Also, right shift of signed int is implementation defined
	acc->vecAcc.data[2] = (int16_t)(ZH << 8 | ZL);
	
	acc->vecAcc.data[0] <<= 16 - FIX_BITS_BEFORE_DOT; // CAN DO MORE
	acc->vecAcc.data[1] <<= 16 - FIX_BITS_BEFORE_DOT;
	acc->vecAcc.data[2] <<= 16 - FIX_BITS_BEFORE_DOT;

	/* DEBUG
	switch(acc->FS) {
		case G_2 : vec3fMult(&acc->vecAcc, ACC_LSB_2G) ; break;
		case G_4 : vec3fMult(&acc->vecAcc, ACC_LSB_4G) ; break;
		case G_8 : vec3fMult(&acc->vecAcc, ACC_LSB_8G) ; break;
		case G_16: vec3fMult(&acc->vecAcc, ACC_LSB_16G); break;
	}
	*/
	acc->flagNewAvail = 0;
}

void magSetDefault(struct magn * mag) 
{
	mag->ODR = m_HZ_75;
	mag->FS  = GAUSS_1x3;

	vec3fZero(&mag->vecMag);
	mag->timeUsed = 0;
	mag->flagNewAvail = 0;
}

void magApply(struct magn * mag)
{		
	char byte = 0x00;
	switch(mag->ODR) {
		case m_HZ_0x75: byte |= MAG_SUB_CRA_MASK_ODR_0x75HZ; break;
		case m_HZ_1x50: byte |= MAG_SUB_CRA_MASK_ODR_1x50HZ; break;
		case m_HZ_3x00: byte |= MAG_SUB_CRA_MASK_ODR_3x00HZ; break;
		case m_HZ_7x50: byte |= MAG_SUB_CRA_MASK_ODR_7x50HZ; break;
		case m_HZ_15  : byte |= MAG_SUB_CRA_MASK_ODR_15HZ  ; break;
		case m_HZ_30  : byte |= MAG_SUB_CRA_MASK_ODR_30HZ  ; break;
		case m_HZ_75  : byte |= MAG_SUB_CRA_MASK_ODR_75HZ  ; break;
		case m_HZ_220 : byte |= MAG_SUB_CRA_MASK_ODR_220HZ ; break;
	}
	I2CWriteReg(MAG_SAD, MAG_SUB_CRA, byte);

	byte = 0x00;
	switch(mag->FS) {
		case GAUSS_1x3: byte |= MAG_SUB_CRB_MASK_FS_1x3GAUSS ; break;
		case GAUSS_1x9: byte |= MAG_SUB_CRB_MASK_FS_1x9GAUSS ; break;
		case GAUSS_2x5: byte |= MAG_SUB_CRB_MASK_FS_2x5GAUSS ; break;
		case GAUSS_4x0: byte |= MAG_SUB_CRB_MASK_FS_4x0GAUSS ; break;
		case GAUSS_4x7: byte |= MAG_SUB_CRB_MASK_FS_4x7GAUSS ; break;
		case GAUSS_5x6: byte |= MAG_SUB_CRB_MASK_FS_5x6GAUSS ; break;
		case GAUSS_8x1: byte |= MAG_SUB_CRB_MASK_FS_8x1GAUSS ; break;
	}
	I2CWriteReg(MAG_SAD, MAG_SUB_CRB, byte);

	byte = 0x00;
	byte |= MAG_SUB_MR_MASK_ENABLE;
	I2CWriteReg(MAG_SAD, MAG_SUB_MR, byte);
}

void magGetAvailability(struct magn * mag)
{	
	unsigned int timeLim = 0;
	switch(mag->ODR) {
		case m_HZ_0x75: timeLim = 2000; break; // Treated as 0.5Hz
		case m_HZ_1x50: timeLim = 1000; break; // Treated as 1Hz
		case m_HZ_3x00: timeLim = 500 ; break; // Treated as 2Hz
		case m_HZ_7x50: timeLim = 200 ; break; // Treated as 5Hz
		case m_HZ_15  : timeLim = 100 ; break; // Treated as 10Hz
		case m_HZ_30  : timeLim = 40  ; break; // Treated as 25Hz
		case m_HZ_75  : timeLim = 17  ; break; // Treated as 60Hz, though closer to 55Hz
		case m_HZ_220 : timeLim = 7   ; break; // Treated as 150Hz, though closer to 125Hz
	}

	if (millis() - mag->timeUsed > timeLim) {
		mag->flagNewAvail = 1;
		mag->timeUsed = millis();
	}
}

void magUpdate(struct magn * mag)
{	
	I2CReadRegSeries(MAG_SAD, MAG_SUB_OUT, 6);
	unsigned int XH = Wire.read();
	unsigned int XL = Wire.read();
	unsigned int ZH = Wire.read();
	unsigned int ZL = Wire.read();
	unsigned int YH = Wire.read();
	unsigned int YL = Wire.read();

	mag->vecMag.data[0] = (int16_t)(XH << 8 | XL);
	mag->vecMag.data[1] = (int16_t)(YH << 8 | YL);
	mag->vecMag.data[2] = (int16_t)(ZH << 8 | ZL);
	
	mag->vecMag.data[0] <<= 16 - FIX_BITS_BEFORE_DOT;
	mag->vecMag.data[1] <<= 16 - FIX_BITS_BEFORE_DOT;
	mag->vecMag.data[2] <<= 16 - FIX_BITS_BEFORE_DOT;

	struct vec3f vecBias;
	vec3fSet(&vecBias, -MAG_BIAS_X, -MAG_BIAS_Y, -MAG_BIAS_Z);
	vec3fAccum(&mag->vecMag, &vecBias);

	struct vec3f vecGain;
	vec3fSet(&vecGain, MAG_GAIN_X, MAG_GAIN_Y, MAG_GAIN_Z);
	vec3fDivVec(&mag->vecMag, &vecGain);

	// vec3fPrint(&mag->vecMag);
	// Serial.printf("\n\r");

	mag->flagNewAvail = 0;
}
