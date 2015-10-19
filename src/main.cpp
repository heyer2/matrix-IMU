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
#define GYR_FREQUENCY 50

#define GYR_OFFSET_SAMPLES 100
#define GYR_LSB_RAD 0.00015271630955 // / 10000

#define SERIAL_BAUD 115200

#define ORTHO_FIX_INTERVAL 10

	
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

void gyrUpdate(struct vec3 * vec, struct vec3 * vecOff)
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
	vec3Add(vecOff, vec, vec);
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

void gyrGetOffset(struct vec3 * vecOff)
{	
	struct vec3 vecTmp;
	struct vec3 vecZero;
	vec3Zero(&vecZero);
	vec3Zero(vecOff);
	for (int i = 0; i < GYR_OFFSET_SAMPLES; i++) {
		gyrUpdate(&vecTmp, &vecZero);
		vec3Add(&vecTmp, vecOff, vecOff);
		delay(1000.0f / GYR_FREQUENCY);
	}
	vec3MultFac(vecOff, -1.0f / GYR_OFFSET_SAMPLES);
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
	gyrInit();
	delay(1000);
}

void loop()
{	
	static float timeElapsed;
	timeElapsed = timeSinceLastCall();
	while(timeElapsed < 1.0f / GYR_FREQUENCY)
		timeElapsed += timeSinceLastCall();
	
	static struct mat3 matOri;
	static struct vec3 vecGyrOff;
	static int firstRun = 1;
	if (firstRun) {
		gyrGetOffset(&vecGyrOff);
		mat3Eyes(&matOri);
		firstRun = 0;
		Serial.printf("Doing stuff");
	}
	
	struct vec3 vecGyrData;
	struct vec3 vecMagData;
	struct vec3 vecAccData;

	
	gyrUpdate(&vecGyrData, &vecGyrOff);
	mat3RotByGyr(&vecGyrData, &matOri, timeElapsed);

	struct vec3 vecUniVert;
	struct vec3 vecOriVert;
	struct mat3 matOriTran;
	mat3Transpose(&matOri, &matOriTran);
	vec3Set(&vecUniVert, 0, 0, 1);
	mat3MultVec(&matOriTran, &vecUniVert, &vecOriVert);
	//vec3Print(&vecOriVert);
	//Serial.printf(" | ");

	accUpdate(&vecAccData);
	if (vec3Length(&vecAccData) > 12000) {
		//Serial.printf("Accb: %3.2f ", vec3Length(&vecAccData));
		vec3Norm(&vecAccData);
		//Serial.printf("Acca: %3.2f ", vec3Length(&vecAccData));
		//Serial.printf("vecOV: %3.2f ", vec3Length(&vecOriVert));
		//vec3Print(&vecAccData);
		//Serial.printf(" | ");
		//Serial.printf("Orit: %3.2f ", mat3Det(&matOriTran));

		float theta = vec3GetAng(&vecOriVert, &vecAccData);
		//Serial.printf("Thet: %3.2f ", theta);
		struct mat3 matRotFix;
		mat3RotFromVecPair(&vecOriVert, &vecAccData, &matRotFix, theta / 100);
		//Serial.printf("Rotfix: %3.2f ", mat3Det(&matRotFix));

		mat3Transpose(&matRotFix, &matRotFix);
		struct mat3 matOriFix;
		mat3Mult(&matOri, &matRotFix, &matOriFix);
		//Serial.printf("MatOrific: %3.2f ", mat3Det(&matOriFix));
		mat3Transpose(&matOriFix, &matOriTran);
		//mat3MultVec(&matOriTran, &vecUniVert, &vecOriVert);

		matOri = matOriFix;
		//Serial.printf(" | ");
	//vec3Print(&vecOriVert);
	//Serial.printf(" | ");
	//Serial.printf("%f", theta);
	//Serial.printf("\n\r");
	}
	//Serial.printf(" | ");
	mat3Print(&matOri);

	Serial.printf("\n\r");
	Serial.send_now();

	static int runsSinceOrthoFix = 0;
	if (runsSinceOrthoFix >= ORTHO_FIX_INTERVAL) {
		mat3OrthoFix(&matOri);
		runsSinceOrthoFix = 0;
	}
	runsSinceOrthoFix++;
}