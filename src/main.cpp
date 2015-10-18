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
#define GYR_FREQUENCY 20

#define GYR_OFFSET_SAMPLES 100
#define GYR_LSB_RAD 0.00015271630955 // / 10000

#define SERIAL_BAUD 115200

#define ORTHO_FIX_INTERVAL 20

	
static struct vec3 vecGyrOff;

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
	gyrGetOffset(&vecGyrOff);
}

void loop()
{	
	static float timeElapsed;
	timeElapsed = timeSinceLastCall();
	while(timeElapsed < 1.0f / GYR_FREQUENCY)
		timeElapsed += timeSinceLastCall();
	
	static mat3 matOri;
	static int firstRun = 1;
	if (firstRun) {
		mat3Eyes(&matOri);
		firstRun = 0;
		Serial.printf("Doing stuff");
	}
	
	struct vec3 vecGyrData;
	struct vec3 vecMagData;
	struct vec3 vecAccData;
	
	gyrUpdate(&vecGyrData, &vecGyrOff);
	mat3RotByGyr(&vecGyrData, &matOri, timeElapsed);
	mat3Print(&matOri);
	Serial.printf("\n\r");
	Serial.send_now();


	static int runsSinceOrthoFix = 0;
	if (runsSinceOrthoFix >= ORTHO_FIX_INTERVAL) {
		mat3OrthoFix(&matOri);
		runsSinceOrthoFix = 0;
	} else
		runsSinceOrthoFix++;

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