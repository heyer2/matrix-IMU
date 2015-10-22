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

#define MAG_BIAS_X (-4.17705880568963)
#define MAG_BIAS_Y (-48.5696006492905)
#define MAG_BIAS_Z 398.835722468053

#define MAG_GAIN_X 0.00234202173981799
#define MAG_GAIN_Y 0.00241234250943554
#define MAG_GAIN_Z 0.00277519214826859

#define GYR_SAD 					 0x6B
#define GYR_SUB_INIT 				 0x20
#define GYR_SUB_INIT_MASK_ENABLE 	 0x0F
#define GYR_SUB_INIT_MASK_ODR_100HZ  0x00
#define GYR_SUB_INIT_MASK_ODR_200HZ  0x40
#define GYR_SUB_INIT_MASK_ODR_400HZ  0x80
#define GYR_SUB_INIT_MASK_ODR_800HZ  0xC0
#define GYR_SUB_INIT_MASK_BW_LOWEST  0x00
#define GYR_SUB_INIT_MASK_BW_LOW 	 0x01
#define GYR_SUB_INIT_MASK_BW_HIGH 	 0x02
#define GYR_SUB_INIT_MASK_BW_HIGHEST 0x03

#define GYR_SUB_OUT (0x28|0x80) // initial register + incrementing output
#define GYR_SUB_STATUS 0x27
#define GYR_SUB_STATUS_MASK_NEWSET 0x04


//useless
#define GYR_FREQUENCY 1000


#define GYR_OFFSET_SAMPLES 1000
#define GYR_LSB_RAD 0.00015271630955 // / 10000

#define SERIAL_BAUD 115200

#define ORTHO_FIX_INTERVAL 10000


/*
struct gyr {
	int newSet;
	struct vec3 vecData;
	enum ODR {100HZ, 200HZ, 300HZ, 400HZ} ODR;
	enum BW {LOWEST, LOW, HIGH, HIGHEST} BW;
	enum SCALE {245DPS, 500DPS, 2000DPS} SCALE;
}
*/

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

int gyrInit(int ODR)
{	
	char byte = 0x00;

	byte |= GYR_SUB_INIT_MASK_ENABLE;
	byte |= GYR_SUB_INIT_MASK_BW_HIGHEST; // Not much reason to go lower

	if (ODR == 100)
		byte |= GYR_SUB_INIT_MASK_ODR_100HZ;
	else if (ODR == 200)
		byte |= GYR_SUB_INIT_MASK_ODR_200HZ;
	else if (ODR == 400)
		byte |= GYR_SUB_INIT_MASK_ODR_400HZ;
	else if (ODR == 800)
		byte |= GYR_SUB_INIT_MASK_ODR_800HZ;
	else
		return 0;

	Wire.beginTransmission(GYR_SAD);
	Wire.write(GYR_SUB_INIT);
	Wire.write(byte);
	Wire.endTransmission(1);
	return 1;
}

void gyrUpdate(struct vec3 * vec, struct vec3 * vecBias)
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
	
	vec3MultFac(vec, GYR_LSB_RAD);
	vec3Add(vecBias, vec, vec);
}

int gyrAvailable()
{
	Wire.beginTransmission(GYR_SAD);
	Wire.write(GYR_SUB_STATUS);
	Wire.endTransmission(0);
	Wire.requestFrom(GYR_SAD, 1);
	char status = Wire.read();
	return status & GYR_SUB_STATUS_MASK_NEWSET;
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

void gyrGetOffset(struct vec3 * vecBias)
{	
	struct vec3 vecTmp;
	struct vec3 vecZero;
	vec3Zero(&vecZero);
	vec3Zero(vecBias);
	for (int i = 0; i < GYR_OFFSET_SAMPLES; i++) {
		gyrUpdate(&vecTmp, &vecZero);
		vec3Add(&vecTmp, vecBias, vecBias);
		delay(1000.0f / GYR_FREQUENCY);
	}
	vec3MultFac(vecBias, -1.0f / GYR_OFFSET_SAMPLES);
}

float timeSinceLastCall()
{	
	static int timeOld;
	int timeNew = millis();
	int timeElapsed = timeNew - timeOld;
	timeOld = timeNew;
	return (float)timeElapsed / 1000;
}

void setup()
{
	Serial.begin(SERIAL_BAUD);
	Wire.begin();
	accInit();
	magInit();
	gyrInit(800);
	delay(1000);
}

void loop()
{	
	static float timeElapsed;
	timeElapsed = timeSinceLastCall();
	while(timeElapsed < 1.0f / GYR_FREQUENCY)
		timeElapsed += timeSinceLastCall();
	
	struct vec3 vecGyrData;
	struct vec3 vecMagData;
	struct vec3 vecAccData;
	static struct mat3 matOri;
	static struct vec3 vecGyrBias;
	static float accGravSize;
	static int firstRun = 1;
	if (firstRun) {
		gyrGetOffset(&vecGyrBias);
		accUpdate(&vecAccData);
		accGravSize = vec3Length(&vecAccData);
		mat3Eyes(&matOri);
		firstRun = 0;
	}
	while (!gyrAvailable());
	gyrUpdate(&vecGyrData, &vecGyrBias);
	//vec3Print(&vecGyrData);
	//Serial.printf(" After: %i ", gyrAvailable()); 
	mat3RotByGyr(&vecGyrData, &matOri, timeElapsed);
	


	//vec3Print(&vecOriVert);
	//Serial.printf(" | ");
	/*
	accUpdate(&vecAccData);
	
	if (accGetDeviation(&vecAccData, accGravSize) < 0.3) {
		struct vec3 vecVert;
		mat3ExtractRow(&matOri, &vecVert, 2);
		vec3Norm(&vecAccData); 
		float theta = vec3GetAng(&vecVert, &vecAccData);
		struct mat3 matRotFix;
		mat3RotFromVecPair(&vecVert, &vecAccData, &matRotFix, -theta / 100);
		mat3Mult(&matOri, &matRotFix, &matOri);
	}

	magUpdate(&vecMagData);
	struct vec3 vecOriNorth;
	mat3MultVec(&matOri, &vecMagData, &vecOriNorth);
	float theta = atan2(vecOriNorth.data[1], vecOriNorth.data[0]);
	struct mat3 matRotFix;
	mat3RotZ(&matRotFix, -theta / 20);
	mat3Mult(&matRotFix, &matOri, &matOri);
	*/
	
	//mat3Print(&matOri);
	//Serial.send_now();
	//vec3Print(&vecGyrData);
	//Serial.printf("Det: %9.8f ", mat3Det(&matOri));
	//Serial.write((char*)&matOri, 4*9);
	
	//mat3Print(&matOri)
	//vec3MultFac(&vecGyrData, 57.29577951308233);
	//vec3Print(&vecGyrData);
	Serial.printf("\n\r");
	Serial.send_now();

	
	static int runsSinceOrthoFix = 0;
	if (runsSinceOrthoFix >= ORTHO_FIX_INTERVAL) {
		//Serial.printf("Det: %4.3f ", mat3Det(&matOri));
		//Serial.printf("\n\r");
		//mat3OrthoFix(&matOri);
		runsSinceOrthoFix = 0;
	}
	runsSinceOrthoFix++;
}