#include "Arduino.h"
#include <matrix.h>
#include <i2c_t3.h> // If using Teensy 3.x
//#include "Wire.h" // If using arduino
#include <sensors.h>


#define SERIAL_BAUD 115200

void setup()
{
	Serial.begin(SERIAL_BAUD);
	delay(400);
	Wire.begin();
	Wire.setRate(I2C_RATE_400);
}

void loop()
{	
	static struct gyro gyr;
	static struct acce acc;
	static struct magn mag;
	static struct mat3 matOri;

	static int flagFirstRun = 1;
	if (flagFirstRun) {
		gyrSetDefault(&gyr);
		gyrApply(&gyr);
		gyrGetBias(&gyr);
		accSetDefault(&acc);
		accApply(&acc);
		magSetDefault(&mag);
		magApply(&mag);
		mat3Eye(&matOri);
		flagFirstRun = 0;
	};
	
	int update = 1;

	gyrGetAvailability(&gyr);
	if (gyr.flagNewAvail) {
		gyrUpdate(&gyr);
		//mat3GyrRot(&gyr.vecGyr, &matOri, gyrTimeSinceUse(&gyr, 1));
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
  		mat3SetColumn(&gyr.vecGyr, &matOri, 1);
  		mat3Send(&matOri);
 	 	Serial.send_now();
  	}
}