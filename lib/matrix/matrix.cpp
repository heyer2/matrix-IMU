//#include <math.h>
#include <matrix.h>
#include "arduino.h"

void mat3Print(struct mat3 * mat)
{
	for (int i = 0; i < 9; i++)
		Serial.printf("%f ", *((float*)mat->data + i));
}

void vec3Print(struct vec3 * vec)
{
	Serial.printf("%f %f %f", vec->data[0], vec->data[1], vec->data[2]);
}

void vec3Zero(struct vec3 * vec)
{
	vec->data[0] = 0;
	vec->data[1] = 0;
	vec->data[2] = 0;
}

void vec3MultFac(struct vec3 * vec, float k)
{
	vec->data[0] *= k;
	vec->data[1] *= k;
	vec->data[2] *= k;
}

void mat3Zero(struct mat3 * mat)
{
	for (int i = 0; i < 9; i++)
		*((float*)mat->data + i) = 0;	
}

void mat3Eyes(struct mat3 * mat)
{
	mat3Zero(mat);
	mat->data[0][0] = 1;
	mat->data[1][1] = 1;
	mat->data[2][2] = 1;
}

void mat3SetRow(int row, struct mat3 * mat, float a, float b, float c)
{
	mat->data[row][0] = a;
	mat->data[row][1] = b;
	mat->data[row][2] = c;
}

void mat3Cpy(struct mat3 * matSource, struct mat3 * matTarget)
{
	*matTarget = *matSource;
}

void mat3Mult(struct mat3 * matA, struct mat3 * matB, struct mat3 * matC)
{	
	struct mat3 matTemp;
	mat3Zero(&matTemp);
	
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			for (int k = 0; k < 3; k++)
				matTemp.data[i][j] += matA->data[i][k] * matB->data[k][j];
	mat3Cpy(&matTemp, matC);
}

void mat3RotByGyr(struct vec3 * vecVel, struct mat3 * matOri, float timeElapsed)
{	
	struct vec3 vecRot = *vecVel;
	vec3MultFac(&vecRot, timeElapsed);
	
	struct mat3 matRotX;
	struct mat3 matRotY;
	struct mat3 matRotZ;
	
	mat3SetRow(0, &matRotX, 1, 0, 0);
	mat3SetRow(1, &matRotX, 0, cos(vecRot.data[0]), -sin(vecRot.data[0]));
	mat3SetRow(2, &matRotX, 0, sin(vecRot.data[0]),  cos(vecRot.data[0]));
	
	mat3SetRow(0, &matRotY,  cos(vecRot.data[1]), 0, sin(vecRot.data[1]));
	mat3SetRow(1, &matRotY, 0, 1, 0);
	mat3SetRow(2, &matRotY, -sin(vecRot.data[1]), 0, cos(vecRot.data[1]));
	
	mat3SetRow(0, &matRotZ, cos(vecRot.data[2]), -sin(vecRot.data[2]), 0);
	mat3SetRow(1, &matRotZ, sin(vecRot.data[2]),  cos(vecRot.data[2]), 0);
	mat3SetRow(2, &matRotZ, 0, 0, 1);
	
	mat3Mult(&matRotX, matOri, matOri);
	mat3Mult(&matRotY, matOri, matOri);
	mat3Mult(&matRotZ, matOri, matOri);
}