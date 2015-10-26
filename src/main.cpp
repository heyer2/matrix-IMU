#include "Arduino.h"
#include <matrix.h>
//#include <i2c_t3.h> // If using Teensy 3.x
#include "Wire.h" // If using arduino

#define int16 short
#define SERIAL_BAUD 115200
#define ORTHO_FIX_INTERVAL 10000
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

#define GYR_BIAS_SAMPLES 1000


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

#define ACC_ALIGN_MAX   0.01 // Max jump in radians
#define ACC_ALIGN_SPEED 0.05 // Defines alignment speed

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

#define MAG_BIAS_X (-4.17705880568963)
#define MAG_BIAS_Y (-48.5696006492905)
#define MAG_BIAS_Z (398.835722468053 )

#define MAG_GAIN_X 0.00234202173981799
#define MAG_GAIN_Y 0.00241234250943554
#define MAG_GAIN_Z 0.00277519214826859

#define MAG_ALIGN_MAX   0.01 // Max jump in radians
#define MAG_ALIGN_SPEED 0.05 // Defines alignment speed


enum gyrOutputRate {g_HZ_100, g_HZ_200, g_HZ_400, g_HZ_800};
enum gyrBandWidth {BW_LOWEST, BW_LOW, BW_HIGH, BW_HIGHEST};
enum gyrFullScale {DPS_245, DPS_500, DPS_2000};
struct gyro {
	int flagNewAvail;
	int timeUsed;
	struct vec3 vecGyr;
	struct vec3 vecBias;
	enum gyrOutputRate ODR;
	enum gyrBandWidth BW;
	enum gyrFullScale FS;
};

enum accOutputRate {a_HZ_1, a_HZ_10, a_HZ_25, a_HZ_50, a_HZ_100, a_HZ_200, a_HZ_400}; // More options exist, but they aren't useful
enum accFullScale {G_2, G_4, G_8, G_16};
struct acce {
	int flagNewAvail;
	struct vec3 vecAcc;
	enum accOutputRate ODR;
	enum accFullScale FS;
};

enum magOutputRate {m_HZ_0x75, m_HZ_1x50, m_HZ_3x00, m_HZ_7x50, m_HZ_15, m_HZ_30, m_HZ_75, m_HZ_220};
enum magFullScale {GAUSS_1x3, GAUSS_1x9, GAUSS_2x5, GAUSS_4x0, GAUSS_4x7, GAUSS_5x6, GAUSS_8x1};
struct magn {
	int flagNewAvail;
	int timeUsed;
	struct vec3 vecMag;
	enum magOutputRate ODR;
	enum magFullScale FS;
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
	return Wire.read();
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
	gyr->ODR = g_HZ_400;
	gyr->BW  = BW_HIGHEST;
	gyr->FS  = DPS_2000;

	vec3Zero(&gyr->vecBias);
	vec3Zero(&gyr->vecGyr);
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

void gyrUpdate(struct gyro * gyr)
{	
	I2CReadRegSeries(GYR_SAD, GYR_SUB_OUT, 6);
	unsigned int XL = Wire.read();
	unsigned int XH = Wire.read();
	unsigned int YL = Wire.read();
	unsigned int YH = Wire.read();
	unsigned int ZL = Wire.read();
	unsigned int ZH = Wire.read();	
	
	gyr->vecGyr.data[0] = (int16)(XH << 8 | XL);
	gyr->vecGyr.data[1] = (int16)(YH << 8 | YL);
	gyr->vecGyr.data[2] = (int16)(ZH << 8 | ZL);
	
	switch(gyr->FS) {
		case DPS_245 : vec3Mult(&gyr->vecGyr, GYR_LSB_245DPS) ; break;
		case DPS_500 : vec3Mult(&gyr->vecGyr, GYR_LSB_500DPS) ; break;
		case DPS_2000: vec3Mult(&gyr->vecGyr, GYR_LSB_2000DPS); break;
	}

	vec3Accum(&gyr->vecGyr, &gyr->vecBias);

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
		vec3Accum(&vecTmp, &gyr->vecGyr);
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

void accSetDefault(struct acce * acc) 
{
	acc->ODR = a_HZ_25;
	acc->FS  = G_2;

	vec3Zero(&acc->vecAcc);
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
	I2CReadRegSeries(ACC_SAD, ACC_SUB_OUT, 6);
	unsigned int XL = Wire.read();
	unsigned int XH = Wire.read();
	unsigned int YL = Wire.read();
	unsigned int YH = Wire.read();
	unsigned int ZL = Wire.read();
	unsigned int ZH = Wire.read();	
	
	acc->vecAcc.data[0] = (int16)(XH << 8 | XL) >> 4; // The value is stored in the upper 12 bits
	acc->vecAcc.data[1] = (int16)(YH << 8 | YL) >> 4; // Also, right shift of signed int is implementation defined
	acc->vecAcc.data[2] = (int16)(ZH << 8 | ZL) >> 4;
	
	switch(acc->FS) {
		case G_2 : vec3Mult(&acc->vecAcc, ACC_LSB_2G) ; break;
		case G_4 : vec3Mult(&acc->vecAcc, ACC_LSB_4G) ; break;
		case G_8 : vec3Mult(&acc->vecAcc, ACC_LSB_8G) ; break;
		case G_16: vec3Mult(&acc->vecAcc, ACC_LSB_16G); break;
	}
	acc->flagNewAvail = 0;
}

void magSetDefault(struct magn * mag) 
{
	mag->ODR = m_HZ_30;
	mag->FS  = GAUSS_1x3;

	vec3Zero(&mag->vecMag);
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
	int timeLim = -1;
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

	mag->vecMag.data[0] = (int16)(XH << 8 | XL);
	mag->vecMag.data[1] = (int16)(YH << 8 | YL);
	mag->vecMag.data[2] = (int16)(ZH << 8 | ZL);
	
	// These are preset, because only a sigle calibration was done
	mag->vecMag.data[0] = (mag->vecMag.data[0] - MAG_BIAS_X) * MAG_GAIN_X;
	mag->vecMag.data[1] = (mag->vecMag.data[1] - MAG_BIAS_Y) * MAG_GAIN_Y;
	mag->vecMag.data[2] = (mag->vecMag.data[2] - MAG_BIAS_Z) * MAG_GAIN_Z;

	mag->flagNewAvail = 0;
}

void setup()
{
	Serial.begin(SERIAL_BAUD);
	Wire.begin();
	delay(400);
	Serial.printf("lol1");
}

void loop()
{	
	static struct gyro gyr;
	static struct acce acc;
	static struct magn mag;
	static struct mat3 matOri;

	static int update;

	static int flagFirstRun = 1;
	if (flagFirstRun) {
		Serial.printf("lol2");
		gyrSetDefault(&gyr);
		Serial.printf("lol3");
		gyrApply(&gyr);
		Serial.printf("lol4");
		gyrGetBias(&gyr);
		Serial.printf("lol5");
		accSetDefault(&acc);
		accApply(&acc);
		Serial.printf("lol6");
		magSetDefault(&mag);
		magApply(&mag);
		Serial.printf("lol7");
		mat3Eyes(&matOri);
		flagFirstRun = 0;
		Serial.printf("lol8");
	};
	
	update = 0;

	gyrGetAvailability(&gyr);
	if (gyr.flagNewAvail) {
		gyrUpdate(&gyr);
		mat3GyrRot(&gyr.vecGyr, &matOri, gyrTimeSinceUse(&gyr, 1));
		update = 1;
	}
	
	accGetAvailability(&acc);
	if (acc.flagNewAvail) {
  		accUpdate(&acc);
  		mat3AccAlign(&acc.vecAcc, &matOri, ACC_ALIGN_SPEED, ACC_ALIGN_MAX);
  		update = 1;
  	}
  	
  	magGetAvailability(&mag);
	if (mag.flagNewAvail) {
  		magUpdate(&mag);
  		mat3MagAlign(&mag.vecMag, &matOri, MAG_ALIGN_SPEED, MAG_ALIGN_MAX);
  		update = 1;
  	}

  	if (update) {
  	mat3Print(&matOri);
  	Serial.printf("\n\r");
  	Serial.send_now();
  	}
}