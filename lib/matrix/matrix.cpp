#include <math.h>
#include <matrix.h>
#include "arduino.h"

void mat3Print(struct mat3 * mat)
{
	for (int i = 0; i < 9; i++)
		Serial.printf("%f", *((float*)mat->data + i));
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

void mat3SetRow(struct mat3 * mat, int row, float a, float b, float c)
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

void mat3RotFromGyro(struct vec3 * vec, struct mat3 * matRot)
{
	struct mat3 matRotX;
	struct mat3 matRotY;
	struct mat3 matRotZ;
	
	mat3SetRow(&matRotX, 0, 1, 0, 0);
	mat3SetRow(&matRotX, 1, 0, cos(vec->data[0]), -sin(vec->data[0]));
	mat3SetRow(&matRotX, 2, 0, sin(vec->data[0]),  cos(vec->data[0]));
	
	mat3SetRow(&matRotY, 0,  cos(vec->data[1]), 0, sin(vec->data[1]));
	mat3SetRow(&matRotY, 1, 0, 1, 0);
	mat3SetRow(&matRotY, 2, -sin(vec->data[1]), 0, cos(vec->data[1]));
	
	mat3SetRow(&matRotZ, 0, cos(vec->data[2]), -sin(vec->data[2]), 0);
	mat3SetRow(&matRotZ, 1, sin(vec->data[2]),  cos(vec->data[2]), 0);
	mat3SetRow(&matRotZ, 2, 0, 0, 1);
	
	mat3Mult(&matRotY, &matRotX, matRot);
	mat3Mult(&matRotZ, matRot, matRot);
}