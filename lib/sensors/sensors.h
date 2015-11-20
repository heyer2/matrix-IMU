#ifndef SENSORS_H
#define SENSORS_H

#include <matrixFix.h>

#define MAG_ALIGN_MAX   float2Fix(0.01) // Max jump in radians
#define MAG_ALIGN_SPEED float2Fix(0.05) // Defines alignment speed
#define ACC_ALIGN_MAX   float2Fix(0.01) // Max jump in radians
#define ACC_ALIGN_SPEED float2Fix(0.05) // Defines alignment speed

enum gyrOutputRate {g_HZ_100, g_HZ_200, g_HZ_400, g_HZ_800};
enum gyrBandWidth {BW_LOWEST, BW_LOW, BW_HIGH, BW_HIGHEST};
enum gyrFullScale {DPS_245, DPS_500, DPS_2000};
struct gyro {
	int flagNewAvail;
	unsigned int timeUsed;
	struct vec3f vecGyr;
	struct vec3f vecAng;
	struct vec3f vecBias;
	enum gyrOutputRate ODR;
	enum gyrBandWidth BW;
	enum gyrFullScale FS;
};

enum accOutputRate {a_HZ_1, a_HZ_10, a_HZ_25, a_HZ_50, a_HZ_100, a_HZ_200, a_HZ_400}; // More options exist, but they aren't useful
enum accFullScale {G_2, G_4, G_8, G_16};
struct acce {
	int flagNewAvail;
	struct vec3f vecAcc;
	enum accOutputRate ODR;
	enum accFullScale FS;
};

enum magOutputRate {m_HZ_0x75, m_HZ_1x50, m_HZ_3x00, m_HZ_7x50, m_HZ_15, m_HZ_30, m_HZ_75, m_HZ_220};
enum magFullScale {GAUSS_1x3, GAUSS_1x9, GAUSS_2x5, GAUSS_4x0, GAUSS_4x7, GAUSS_5x6, GAUSS_8x1};
struct magn {
	int flagNewAvail;
	unsigned int timeUsed;
	struct vec3f vecMag;
	enum magOutputRate ODR;
	enum magFullScale FS;
};


void gyrSetDefault(struct gyro * gyr);
void gyrApply(struct gyro * gyr);
void gyrGetAvailability(struct gyro * gyr);
void gyrUpdate(struct gyro * gyr);
void gyrGetBias(struct gyro * gyr);
float gyrTimeSinceUse(struct gyro * gyr, int update);
void accSetDefault(struct acce * acc);
void accApply(struct acce * acc);
void accGetAvailability(struct acce * acc);
void accUpdate(struct acce * acc);
void magSetDefault(struct magn * mag);
void magApply(struct magn * mag);
void magGetAvailability(struct magn * mag);
void magUpdate(struct magn * mag);

#endif /* SENSOR_H */