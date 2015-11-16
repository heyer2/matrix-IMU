#include <stdint.h>
#include "arduino.h"
#include <matrixFix.h>


static inline intFix limit(intFix input, intFix min, intFix max)
{
	if (input > max)
		return max;
	else if (input < min)
		return min;
	else
		return input;
}

void vec3Send(struct vec3 * vec)
{
	Serial.write((const char*)&vec->data, 3 * sizeof(intFix));
	Serial.write('\n');
}

void vec3Print(struct vec3 * vec)
{
	Serial.printf("%i %i %i", vec->data[0], vec->data[1], vec->data[2]);
}

void vec3Zero(struct vec3 * vec)
{
	vec->data[0] = 0;
	vec->data[1] = 0;
	vec->data[2] = 0;
}

void vec3Set(struct vec3 * vec, intFix x, intFix y, intFix z)
{
	vec->data[0] = x;
	vec->data[1] = y;
	vec->data[2] = z;	
}

void vec3Mult(struct vec3 * vec, intFix k)
{
	vec->data[0] = fixMult(vec->data[0], k);
	vec->data[1] = fixMult(vec->data[1], k);
	vec->data[2] = fixMult(vec->data[2], k);
}

void vec3MultVec(struct vec3 * vec, struct vec3 * vecFac)
{
	vec->data[0] = fixMult(vec->data[0], vecFac->data[0]);
	vec->data[1] = fixMult(vec->data[1], vecFac->data[1]);
	vec->data[2] = fixMult(vec->data[2], vecFac->data[2]);
}

void vec3Add(struct vec3 * vecA, struct vec3 * vecB, struct vec3 * vecOut)
{
	vecOut->data[0] = vecA->data[0] + vecB->data[0];
	vecOut->data[1] = vecA->data[1] + vecB->data[1];
	vecOut->data[2] = vecA->data[2] + vecB->data[2];
}

void vec3Accum(struct vec3 * vecAccum, struct vec3 * vecAdd)
{
	vecAccum->data[0] += vecAdd->data[0];
	vecAccum->data[1] += vecAdd->data[1];
	vecAccum->data[2] += vecAdd->data[2];
}

void vec3AccumMult(struct vec3 * vecAccum, struct vec3 * vecAdd, intFix k)
{
	vecAccum->data[0] += fixMult(vecAdd->data[0], k);
	vecAccum->data[1] += fixMult(vecAdd->data[1], k);
	vecAccum->data[2] += fixMult(vecAdd->data[2], k);
}

intFix vec3DotProd(struct vec3 * vecA, struct vec3 * vecB)
{
	return fixMult(vecA->data[0], vecB->data[0]) + fixMult(vecA->data[1], vecB->data[1]) + fixMult(vecA->data[2], vecB->data[2]);
}

void vec3CrossProd(struct vec3 * vecA, struct vec3 * vecB, struct vec3 * vecCross)
{
	vecCross->data[0] = fixMult(vecA->data[1], vecB->data[2]) - fixMult(vecA->data[2], vecB->data[1]);
	vecCross->data[1] = fixMult(vecA->data[2], vecB->data[0]) - fixMult(vecA->data[0], vecB->data[2]);
	vecCross->data[2] = fixMult(vecA->data[0], vecB->data[1]) - fixMult(vecA->data[1], vecB->data[0]);
}

intFix vec3Length(struct vec3 * vec)
{
	return sqrtFix(vec3DotProd(vec, vec));
}

void vec3Norm(struct vec3 * vec)
{
	vec3Div(vec, vec3Length(vec));
}

float vec3GetAng(struct vec3 * vecA, struct vec3 * vecB)
{	
	intFix tmp = limit(vec3DotProd(vecA, vecB), 0, 1);
	return acos(fix2Float(tmp));
}

void mat3Send(struct mat3 * mat)
{
	Serial.write((const char*)&mat->data, 9 * sizeof(intFix));
	Serial.write('\n');
}

void mat3Print(struct mat3 * mat)
{
	Serial.printf("%i %i %i %i %i %i %i %i %i",
	    mat->data[0][0], mat->data[0][1], mat->data[0][2],
        mat->data[1][0], mat->data[1][1], mat->data[1][2],
		mat->data[2][0], mat->data[2][1], mat->data[2][2]);
}

void mat3Zero(struct mat3 * mat)
{
	for (int i = 0; i < 9; i++)
		((intFix*)mat->data)[i] = 0;
}

void mat3Eye(struct mat3 * mat)
{
	mat3Zero(mat);
	mat->data[0][0] = FIX_UNITY;
	mat->data[1][1] = FIX_UNITY;
	mat->data[2][2] = FIX_UNITY;
}

void mat3Mult(struct mat3 * matA, struct mat3 * matB, struct mat3 * matOut)
{	
	struct mat3 matTmp;
	mat3Zero(&matTmp);
	
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			for (int k = 0; k < 3; k++)
				matTmp.data[i][j] += fixMult(matA->data[i][k], matB->data[k][j]);
	*matOut = matTmp;
}

void mat3MultVec(struct mat3 * mat, struct vec3 * vecIn, struct vec3 * vecOut)
{
	struct vec3 vecTmp;
	vec3Zero(&vecTmp);
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			vecTmp.data[i] += fixMult(mat->data[i][j], vecIn->data[j]);
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

void mat3SetRowMan(int row, struct mat3 * mat, intFix a, intFix b, intFix c)
{
	mat->data[row][0] = a;
	mat->data[row][1] = b;
	mat->data[row][2] = c;
}

intFix mat3Det(struct mat3 * mat)
{
	intFix result = 0;

	result += fixMult(fixMult(mat->data[0][0], mat->data[1][1]), mat->data[2][2]);
	result += fixMult(fixMult(mat->data[0][1], mat->data[1][2]), mat->data[2][0]);
	result += fixMult(fixMult(mat->data[0][2], mat->data[1][0]), mat->data[2][1]);
	result -= fixMult(fixMult(mat->data[0][1], mat->data[1][0]), mat->data[2][2]);
	result -= fixMult(fixMult(mat->data[0][0], mat->data[1][2]), mat->data[2][1]);
	result -= fixMult(fixMult(mat->data[0][2], mat->data[1][1]), mat->data[2][0]);	

	return result;
}

void mat3RotAxisX(struct mat3 * mat, float theta)
{
	struct vec3 vecY;
	struct vec3 vecZ;
	mat3ExtractColumn(mat, &vecY, 1);
	mat3ExtractColumn(mat, &vecZ, 2);

	struct vec3 vecTmp;

	intFix cosTheta = FIX_UNITY - float2Fix(theta * theta * 0.5);
	intFix sinTheta = float2Fix(theta);

	vec3Zero(&vecTmp);
	vec3AccumMult(&vecTmp, &vecY, cosTheta);
	vec3AccumMult(&vecTmp, &vecZ, sinTheta);
	mat3SetColumn(&vecTmp, mat, 1);

	vec3Zero(&vecTmp);
	vec3AccumMult(&vecTmp, &vecY, -sinTheta);
	vec3AccumMult(&vecTmp, &vecZ,  cosTheta);
	mat3SetColumn(&vecTmp, mat, 2);
}

void mat3RotAxisY(struct mat3 * mat, float theta)
{	
	struct vec3 vecX;
	struct vec3 vecZ;
	mat3ExtractColumn(mat, &vecX, 0);
	mat3ExtractColumn(mat, &vecZ, 2);

	struct vec3 vecTmp;

	intFix cosTheta = FIX_UNITY - float2Fix(theta * theta * 0.5);
	intFix sinTheta = float2Fix(theta);

	vec3Zero(&vecTmp);
	vec3AccumMult(&vecTmp, &vecX,  cosTheta);
	vec3AccumMult(&vecTmp, &vecZ, -sinTheta);
	mat3SetColumn(&vecTmp, mat, 0);

	vec3Zero(&vecTmp);
	vec3AccumMult(&vecTmp, &vecX, sinTheta);
	vec3AccumMult(&vecTmp, &vecZ, cosTheta);
	mat3SetColumn(&vecTmp, mat, 2);
}

void mat3RotAxisZ(struct mat3 * mat, float theta)
{
	struct vec3 vecX;
	struct vec3 vecY;
	mat3ExtractColumn(mat, &vecX, 0);
	mat3ExtractColumn(mat, &vecY, 1);

	struct vec3 vecTmp;

	intFix cosTheta = FIX_UNITY - float2Fix(theta * theta * 0.5);
	intFix sinTheta = float2Fix(theta);

	vec3Zero(&vecTmp);
	vec3AccumMult(&vecTmp, &vecX, cosTheta);
	vec3AccumMult(&vecTmp, &vecY, sinTheta);
	mat3SetColumn(&vecTmp, mat, 0);

	vec3Zero(&vecTmp);
	vec3AccumMult(&vecTmp, &vecX, -sinTheta);
	vec3AccumMult(&vecTmp, &vecY,  cosTheta);
	mat3SetColumn(&vecTmp, mat, 1);
}

void mat3GyrRot(struct vec3 * vecVel, struct mat3 * matOri, float timeElapsed)
{	
	mat3RotAxisX(matOri, vecVel->data[0] * timeElapsed);
	mat3RotAxisY(matOri, vecVel->data[1] * timeElapsed);
	mat3RotAxisZ(matOri, vecVel->data[2] * timeElapsed);
}

void mat3RotFromAxis(struct vec3 * vecAxis, struct mat3 * matRot, float theta)
{	
	intFix cosTheta = float2Fix(cos(theta));
	intFix sinTheta = float2Fix(sin(theta));
	intFix oneMinusCos = 1 - cosTheta;
	
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

void mat3RotFromVecPair(struct vec3 * vecA, struct vec3 * vecB, struct mat3 * matRot, float theta)
{	
	struct vec3 vecAxis;
	vec3CrossProd(vecA, vecB, &vecAxis);
	vec3Norm(&vecAxis);
	mat3RotFromAxis(&vecAxis, matRot, theta);
}

void mat3AccAlign(struct vec3 * vecAcc, struct mat3 * matOri, float weight, float max)
{		
		struct vec3 vecZ;
		struct vec3 vecTmp;
		mat3ExtractRow(matOri, &vecZ, 2);
		vecTmp = *vecAcc;
		vec3Norm(&vecTmp); //Removing this would make the angle slightly wrong, since A.B  = cos(theta)*||A|B|

		float theta = vec3GetAng(&vecZ, &vecTmp);
		theta = limit(theta * weight, -max, max); // Albeit theta can only be positive

		struct mat3 matRotAlign;
		mat3RotFromVecPair(&vecZ, &vecTmp, &matRotAlign, -theta); // Negation can be removed by switching inputs to crossproduct
		mat3Mult(matOri, &matRotAlign, matOri);
}

void mat3RotZ(struct mat3 * matRot, float theta)
{
	mat3SetRowMan(0, matRot, float2Fix(cos(theta)), float2Fix(-sin(theta)), 0);
	mat3SetRowMan(1, matRot, float2Fix(sin(theta)), float2Fix( cos(theta)), 0);
	mat3SetRowMan(2, matRot, 0, 0, 1);
}

void mat3MagAlign(struct vec3 * vecMag, struct mat3 * matOri, float weight, float max)
{	
	struct vec3 vecNorth;
	mat3MultVec(matOri, vecMag, &vecNorth);

	float theta = atan2(vecNorth.data[1], vecNorth.data[0]);
	theta = limit(theta * weight, -max, max);

	struct mat3 matRotAlign;
	mat3RotZ(&matRotAlign, -theta);
	mat3Mult(&matRotAlign, matOri, matOri);
}

void mat3OrthoFix(struct mat3 * mat)
{
	struct vec3 vecOldX;
	struct vec3 vecOldY;
	
	mat3ExtractColumn(mat, &vecOldX, 0);
	mat3ExtractColumn(mat, &vecOldY, 1);
	
	intFix err = vec3DotProd(&vecOldX, &vecOldY);
	struct vec3 vecX = vecOldY;
	vec3Mult(&vecX, (-0.5) * err);
	vec3Accum(&vecX, &vecOldX);
	vec3Norm(&vecX);
	mat3SetColumn(&vecX, mat, 0);
	
	struct vec3 vecY = vecOldX;
	vec3Mult(&vecY, (-0.5) * err);
	vec3Accum(&vecY, &vecOldY);
	vec3Norm(&vecY);
	mat3SetColumn(&vecY, mat, 1);
	
	struct vec3 vecZ;
	vec3CrossProd(&vecX, &vecY, &vecZ);
	mat3SetColumn(&vecZ, mat, 2);
}


