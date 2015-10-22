//#include <math.h>
#include <matrix.h>
#include "arduino.h"

static float limit(float input, float min, float max)
{
	if (input > max)
		return max;
	else if (input < min)
		return min;
	else
		return input;
}

void mat3Print(struct mat3 * mat)
{
	Serial.printf("%4.3f %4.3f %4.3f %4.3f %4.3f %4.3f %4.3f %4.3f %4.3f",
					     mat->data[0][0], mat->data[0][1], mat->data[0][2],
					     mat->data[1][0], mat->data[1][1], mat->data[1][2],
					     mat->data[2][0], mat->data[2][1], mat->data[2][2]);
}

void vec3Print(struct vec3 * vec)
{
	Serial.printf("% 5.3f % 5.3f % 5.3f", vec->data[0], vec->data[1], vec->data[2]);
}

void vec3MultFac(struct vec3 * vec, float k)
{
	vec->data[0] *= k;
	vec->data[1] *= k;
	vec->data[2] *= k;
}

void vec3Zero(struct vec3 * vec)
{
	vec->data[0] = 0;
	vec->data[1] = 0;
	vec->data[2] = 0;
}

void vec3Set(struct vec3 * vec, float x, float y, float z)
{
	vec->data[0] = x;
	vec->data[1] = y;
	vec->data[2] = z;	
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

void mat3Mult(struct mat3 * matA, struct mat3 * matB, struct mat3 * matOut)
{	
	struct mat3 matTmp;
	mat3Zero(&matTmp);
	
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			for (int k = 0; k < 3; k++)
				matTmp.data[i][j] += matA->data[i][k] * matB->data[k][j];
	*matOut = matTmp;
}

void mat3ExtractColumn(struct mat3 * mat, struct vec3 * vec, int col)
{
	vec->data[0] = mat->data[0][col];
	vec->data[1] = mat->data[1][col];
	vec->data[2] = mat->data[2][col];
}

void mat3ExtractRow(struct mat3 * mat, struct vec3 * vec, int row)
{
	vec->data[0] = mat->data[row][0];
	vec->data[1] = mat->data[row][1];
	vec->data[2] = mat->data[row][2];
}

void mat3SetColumn(struct vec3 * vec, struct mat3 * mat, int col)
{
	mat->data[0][col] = vec->data[0];
	mat->data[1][col] = vec->data[1];
	mat->data[2][col] = vec->data[2];
}

void mat3SetRow(int row, struct mat3 * mat, float a, float b, float c)
{
	mat->data[row][0] = a;
	mat->data[row][1] = b;
	mat->data[row][2] = c;
}

void mat3RotFromAxis(struct vec3 * vecAxis, struct mat3 * matRot, float theta)
{	
	float cosTheta = cos(theta);
	float sinTheta = sin(theta);
	float oneMinusCos = 1 - cosTheta;
	
	matRot->data[0][0] = cosTheta + vecAxis->data[0] * vecAxis->data[0] * oneMinusCos;
	matRot->data[0][1] = vecAxis->data[0] * vecAxis->data[1] * oneMinusCos - vecAxis->data[2] * sinTheta;
	matRot->data[0][2] = vecAxis->data[0] * vecAxis->data[2] * oneMinusCos + vecAxis->data[1] * sinTheta;
	matRot->data[1][0] = vecAxis->data[0] * vecAxis->data[1] * oneMinusCos + vecAxis->data[2] * sinTheta;
	matRot->data[1][1] = cosTheta + vecAxis->data[1] * vecAxis->data[1] * oneMinusCos;
	matRot->data[1][2] = vecAxis->data[1] * vecAxis->data[2] * oneMinusCos - vecAxis->data[0] * sinTheta;
	matRot->data[2][0] = vecAxis->data[0] * vecAxis->data[2] * oneMinusCos - vecAxis->data[1] * sinTheta;
	matRot->data[2][1] = vecAxis->data[1] * vecAxis->data[2] * oneMinusCos + vecAxis->data[0] * sinTheta;
	matRot->data[2][2] = cosTheta + vecAxis->data[2] * vecAxis->data[2] * oneMinusCos;
}

void vec3AddMultFac(struct vec3 * vecIn, struct vec3 * vecOut, float k)
{
	vecOut->data[0] += vecIn->data[0] * k;
	vecOut->data[1] += vecIn->data[1] * k;
	vecOut->data[2] += vecIn->data[2] * k;
}

void mat3RotAxisX(struct mat3 * mat, float theta)
{
	struct vec3 vecY;
	struct vec3 vecZ;
	mat3ExtractColumn(mat, &vecY, 1);
	mat3ExtractColumn(mat, &vecZ, 2);

	struct vec3 vecTmp;

	float cosTheta = 1 - theta * theta * 0.5;
	float sinTheta = theta;

	vec3Zero(&vecTmp);
	vec3AddMultFac(&vecY, &vecTmp, cosTheta);
	vec3AddMultFac(&vecZ, &vecTmp, sinTheta);
	mat3SetColumn(&vecTmp, mat, 1);

	vec3Zero(&vecTmp);
	vec3AddMultFac(&vecY, &vecTmp, -sinTheta);
	vec3AddMultFac(&vecZ, &vecTmp, cosTheta);
	mat3SetColumn(&vecTmp, mat, 2);
}

void mat3RotAxisY(struct mat3 * mat, float theta)
{	
	struct vec3 vecX;
	struct vec3 vecZ;
	mat3ExtractColumn(mat, &vecX, 0);
	mat3ExtractColumn(mat, &vecZ, 2);

	struct vec3 vecTmp;

	float cosTheta = 1 - theta * theta * 0.5;
	float sinTheta = theta;

	vec3Zero(&vecTmp);
	vec3AddMultFac(&vecX, &vecTmp, cosTheta);
	vec3AddMultFac(&vecZ, &vecTmp, -sinTheta);
	mat3SetColumn(&vecTmp, mat, 0);

	vec3Zero(&vecTmp);
	vec3AddMultFac(&vecX, &vecTmp, sinTheta);
	vec3AddMultFac(&vecZ, &vecTmp, cosTheta);
	mat3SetColumn(&vecTmp, mat, 2);
}

void mat3RotAxisZ(struct mat3 * mat, float theta)
{
	struct vec3 vecX;
	struct vec3 vecY;
	mat3ExtractColumn(mat, &vecX, 0);
	mat3ExtractColumn(mat, &vecY, 1);

	struct vec3 vecTmp;

	float cosTheta = 1 - theta * theta * 0.5;
	float sinTheta = theta;

	vec3Zero(&vecTmp);
	vec3AddMultFac(&vecX, &vecTmp, cosTheta);
	vec3AddMultFac(&vecY, &vecTmp, sinTheta);
	mat3SetColumn(&vecTmp, mat, 0);

	vec3Zero(&vecTmp);
	vec3AddMultFac(&vecX, &vecTmp, -sinTheta);
	vec3AddMultFac(&vecY, &vecTmp, cosTheta);
	mat3SetColumn(&vecTmp, mat, 1);
}

void mat3RotByGyr(struct vec3 * vecVel, struct mat3 * matOri, float timeElapsed)
{	
	/*
	struct mat3 matRotX;
	struct mat3 matRotY;
	struct mat3 matRotZ;
	struct vec3 vecX;
	struct vec3 vecY;
	struct vec3 vecZ;
	
	mat3ExtractColumn(matOri, &vecX, 0);
	mat3ExtractColumn(matOri, &vecY, 1);
	mat3ExtractColumn(matOri, &vecZ, 2);
	
	mat3RotFromAxis(&vecX, &matRotX, vecVel->data[0] * timeElapsed);
	mat3RotFromAxis(&vecY, &matRotY, vecVel->data[1] * timeElapsed);
	mat3RotFromAxis(&vecZ, &matRotZ, vecVel->data[2] * timeElapsed);
	
	mat3Mult(&matRotX, matOri, matOri);
	mat3Mult(&matRotY, matOri, matOri);
	mat3Mult(&matRotZ, matOri, matOri);
	*/

	mat3RotAxisX(matOri, vecVel->data[0] * timeElapsed);
	mat3RotAxisY(matOri, vecVel->data[1] * timeElapsed);
	mat3RotAxisZ(matOri, vecVel->data[2] * timeElapsed);
}


float vec3DotProd(struct vec3 * vecA, struct vec3 * vecB)
{
	return vecA->data[0] * vecB->data[0] + vecA->data[1] * vecB->data[1] + vecA->data[2] * vecB->data[2];
}

void vec3CrossProd(struct vec3 * vecA, struct vec3 * vecB, struct vec3 * vecCross)
{
	vecCross->data[0] = vecA->data[1] * vecB->data[2] - vecA->data[2] * vecB->data[1];
	vecCross->data[1] = vecA->data[2] * vecB->data[0] - vecA->data[0] * vecB->data[2];
	vecCross->data[2] = vecA->data[0] * vecB->data[1] - vecA->data[1] * vecB->data[0];
}

void vec3Add(struct vec3 * vecA, struct vec3 * vecB, struct vec3 * vecOut)
{
	vecOut->data[0] = vecA->data[0] + vecB->data[0];
	vecOut->data[1] = vecA->data[1] + vecB->data[1];
	vecOut->data[2] = vecA->data[2] + vecB->data[2];
}

float vec3Length(struct vec3 * vec)
{
	return sqrt(vec3DotProd(vec, vec));
}

void vec3Norm(struct vec3 * vec)
{
	vec3MultFac(vec, 1 / vec3Length(vec));
}


void mat3OrthoFix(struct mat3 * mat)
{
	struct vec3 vecOldX;
	struct vec3 vecOldY;
	
	mat3ExtractColumn(mat, &vecOldX, 0);
	mat3ExtractColumn(mat, &vecOldY, 1);
	
	float err = vec3DotProd(&vecOldX, &vecOldY);
	struct vec3 vecX = vecOldY;
	vec3MultFac(&vecX, (-0.5) * err);
	vec3Add(&vecOldX, &vecX, &vecX);
	vec3Norm(&vecX);
	mat3SetColumn(&vecX, mat, 0);
	
	struct vec3 vecY = vecOldX;
	vec3MultFac(&vecY, (-0.5) * err);
	vec3Add(&vecOldY, &vecY, &vecY);
	vec3Norm(&vecY);
	mat3SetColumn(&vecY, mat, 1);
	
	struct vec3 vecZ;
	vec3CrossProd(&vecX, &vecY, &vecZ);
	mat3SetColumn(&vecZ, mat, 2);
}

void mat3MultVec(struct mat3 * mat, struct vec3 * vecIn, struct vec3 * vecOut)
{
	struct vec3 vecTmp;
	vec3Zero(&vecTmp);
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			vecTmp.data[i] += mat->data[i][j] * vecIn->data[j];
	*vecOut = vecTmp;
}

void mat3Transpose(struct mat3 * matIn, struct mat3 * matOut)
{	
	struct mat3 matTmp = *matIn;
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			matTmp.data[i][j] = matIn->data[j][i];
	*matOut = matTmp;
}


float vec3GetAng(struct vec3 * vecA, struct vec3 * vecB)
{
	return acos(limit(vec3DotProd(vecA, vecB), 0, 1));
}

void mat3RotFromVecPair(struct vec3 * vecA, struct vec3 * vecB, struct mat3 * matRot, float theta)
{	
	struct vec3 vecAxis;
	vec3CrossProd(vecA, vecB, &vecAxis);
	vec3Norm(&vecAxis);
	mat3RotFromAxis(&vecAxis, matRot, theta);
}

float mat3Det(struct mat3 * mat)
{
	float result = 0;

	result += mat->data[0][0] * mat->data[1][1] * mat->data[2][2];
	result += mat->data[0][1] * mat->data[1][2] * mat->data[2][0];
	result += mat->data[0][2] * mat->data[1][0] * mat->data[2][1];

	result -= mat->data[0][1] * mat->data[1][0] * mat->data[2][2];
	result -= mat->data[0][0] * mat->data[1][2] * mat->data[2][1];
	result -= mat->data[0][2] * mat->data[1][1] * mat->data[2][0];	

	return result;
}

void mat3RotZ(struct mat3 * matRot, float theta)
{
	mat3SetRow(0, matRot, cos(theta), -sin(theta), 0);
	mat3SetRow(1, matRot, sin(theta),  cos(theta), 0);
	mat3SetRow(2, matRot, 0, 0, 1);
}

