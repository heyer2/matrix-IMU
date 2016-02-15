#include "Arduino.h"
#include <matrixFix.h>
#include <i2c_t3.h> // If using Teensy 3.x
//#include "Wire.h" // If using arduino
#include <sensors.h>

#define SERIAL_BAUD 115200
#define SERIAL_STARTUP_DELAY 1000 // In milliseconds
#define SERIAL_SEND_INTERVAL 10000 // In microseconds

#define MAG_ALIGN_MAX   float2Fix(0.01) // Max jump in radians
#define MAG_ALIGN_SPEED float2Fix(0.05) // Defines alignment speed
#define ACC_ALIGN_MAX   float2Fix(0.01) // Max jump in radians
#define ACC_ALIGN_SPEED float2Fix(0.01) // Defines alignment speed

#define ORIMAT_DET_ERR_LIM float2Fix(0.005)

void setup()
{
	Serial.begin(SERIAL_BAUD);
	delay(SERIAL_STARTUP_DELAY);
	Wire.begin();
	Wire.setRate(I2C_RATE_400);
}

static unsigned int timeSince(unsigned int timer)
{
	return micros() - timer;
}

void loop()
{
	static struct gyro gyr;
	static struct acce acc;
	static struct magn mag;
	static struct mat3f matOri;

	static unsigned int timerSend = 0;
	static unsigned int intervalSend;

	static int flagFirstRun = 1;
	if (flagFirstRun) {
		gyrSetDefault(&gyr);
		gyrApply(&gyr);
		gyrGetBias(&gyr);
		accSetDefault(&acc);
		accApply(&acc);
		magSetDefault(&mag);
		magApply(&mag);
		mat3fEye(&matOri);
		flagFirstRun = 0;
	};
	
	int update = 0;
	
	gyrGetAvailability(&gyr);
	if (gyr.flagNewAvail) {
		gyrUpdate(&gyr);
		mat3fGyrRot(&gyr.vecAng, &matOri);
		update = 1;
	}
  
	accGetAvailability(&acc);
	if (acc.flagNewAvail) {
		accUpdate(&acc);
		mat3fAccAlign(&acc.vecAcc, &matOri, ACC_ALIGN_SPEED, ACC_ALIGN_MAX);
		update = 1;
 	}
  	
  	magGetAvailability(&mag);
	if (mag.flagNewAvail) {
		magUpdate(&mag);
		mat3fMagAlign(&mag.vecMag, &matOri, MAG_ALIGN_SPEED, MAG_ALIGN_MAX);
		update = 1;
		}
	
 	if (abs(mat3fDet(&matOri) - FIX_UNITY) > ORIMAT_DET_ERR_LIM)
 		mat3fOrthoFix(&matOri);

	if (update) {//timeSince(timerSend) > SERIAL_SEND_INTERVAL
		intervalSend = timeSince(timerSend);
		timerSend += intervalSend;
    	mat3fSend(&matOri);
		Serial.send_now();
  	}
}