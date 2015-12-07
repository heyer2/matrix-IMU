#include "Arduino.h"
#include <matrix.h>
#include <matrixFix.h>
#include <i2c_t3.h> // If using Teensy 3.x
//#include "Wire.h" // If using arduino
#include <sensors.h>

#define SERIAL_BAUD 115200

void setup()
{
	Serial.begin(SERIAL_BAUD);
	delay(1500);
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
	static unsigned int timerCalc = 0;
	static unsigned int intervalSend;
	static unsigned int intervalCalc;

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
	/*
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
	*/
  	if (update) {
  		intervalCalc = timeSince(timerCalc);
  		timerCalc += intervalCalc;
  	}

  	if (timeSince(timerSend) > 10000) {
  		intervalSend = timeSince(timerSend);
  		timerSend += intervalSend;

  		float record = 0;
  		for (int i = -1000; i < 1000; i++)
  		{	
  			intFix ang = FIX_UNITY / 1000 * i;
  			Serial.printf("in: %f sin: %f approx: %f \n\r",fix2Float(ang), fix2Float(fixSin(ang)), fix2Float(fixSin2(ang)));
  			float tmp = abs(fix2Float(fixSin(ang)) - fix2Float(fixSin2(ang)));
  			if (tmp > record)
  				record = tmp;
  		}
  		Serial.printf("Rec: %f \n\r",record);
  		delay(1000);
  		//mat3fSend(&matOri);
 	 	Serial.send_now();
  	}
}