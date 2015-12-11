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

void vec3fSend(struct vec3f * vec)
{
	Serial.write((const char*)&vec->data, 3 * sizeof(intFix));
	Serial.write('\n');
}

void vec3fPrint(struct vec3f * vec)
{
	Serial.printf("% f % f % f ", fix2Float(vec->data[0]), fix2Float(vec->data[1]), fix2Float(vec->data[2]));
}

void vec3fSet(struct vec3f * vec, intFix x, intFix y, intFix z)
{
	vec->data[0] = x;
	vec->data[1] = y;
	vec->data[2] = z;	
}

void vec3fZero(struct vec3f * vec)
{
	vec->data[0] = 0;
	vec->data[1] = 0;
	vec->data[2] = 0;
}

void vec3fAdd(struct vec3f * vecA, struct vec3f * vecB, struct vec3f * vecOut)
{
	vecOut->data[0] = vecA->data[0] + vecB->data[0];
	vecOut->data[1] = vecA->data[1] + vecB->data[1];
	vecOut->data[2] = vecA->data[2] + vecB->data[2];
}

void vec3fAccum(struct vec3f * vecAccum, struct vec3f * vecAdd)
{
	vecAccum->data[0] += vecAdd->data[0];
	vecAccum->data[1] += vecAdd->data[1];
	vecAccum->data[2] += vecAdd->data[2];
}

void vec3fMult(struct vec3f * vec, intFix k)
{
	vec->data[0] = fixMult(vec->data[0], k);
	vec->data[1] = fixMult(vec->data[1], k);
	vec->data[2] = fixMult(vec->data[2], k);
}

void vec3fMultVec(struct vec3f * vecRes, struct vec3f * vec)
{
	vecRes->data[0] = fixMult(vecRes->data[0], vec->data[0]);
	vecRes->data[1] = fixMult(vecRes->data[1], vec->data[1]);
	vecRes->data[2] = fixMult(vecRes->data[2], vec->data[2]);
}

void vec3fAccumMult(struct vec3f * vecAccum, struct vec3f * vecAdd, intFix k)
{
	vecAccum->data[0] += fixMult(vecAdd->data[0], k);
	vecAccum->data[1] += fixMult(vecAdd->data[1], k);
	vecAccum->data[2] += fixMult(vecAdd->data[2], k);
}

void vec3fDiv(struct vec3f * vec, intFix k)
{
	vec->data[0] = fixDiv(vec->data[0], k);
	vec->data[1] = fixDiv(vec->data[1], k);
	vec->data[2] = fixDiv(vec->data[2], k);
}

void vec3fDivVec(struct vec3f * vecRes, struct vec3f * vec)
{
	vecRes->data[0] = fixDiv(vecRes->data[0], vec->data[0]);
	vecRes->data[1] = fixDiv(vecRes->data[1], vec->data[1]);
	vecRes->data[2] = fixDiv(vecRes->data[2], vec->data[2]);
}

longFix vec3fDotProd(struct vec3f * vecA, struct vec3f * vecB)
{	
	longFix tmp = 0;
	tmp += fixMult(vecA->data[0], vecB->data[0]);
	tmp += fixMult(vecA->data[1], vecB->data[1]);
	tmp += fixMult(vecA->data[2], vecB->data[2]);
 	return tmp;
}

void vec3fCrossProd(struct vec3f * vecA, struct vec3f * vecB, struct vec3f * vecCross)
{
	vecCross->data[0] = fixMult(vecA->data[1], vecB->data[2]) - fixMult(vecA->data[2], vecB->data[1]);
	vecCross->data[1] = fixMult(vecA->data[2], vecB->data[0]) - fixMult(vecA->data[0], vecB->data[2]);
	vecCross->data[2] = fixMult(vecA->data[0], vecB->data[1]) - fixMult(vecA->data[1], vecB->data[0]);
}

intFix vec3fLength(struct vec3f * vec)
{	
	return fixSqrt(vec3fDotProd(vec, vec));
}

void vec3fNorm(struct vec3f * vec)
{	
	vec3fDiv(vec, vec3fLength(vec));
}

static void vec3fUnitVecEnhance(struct vec3f * vec) 
{
	intFix dot = vec3fDotProd(vec, vec); //intfix because it can be represented as such, the length is around 1
	intFix lengthInverse = (FIX_UNITY + (FIX_UNITY >> 1)) - (dot >> 1); // 1/sqrt(x) approximation for precision
	vec3fMult(vec, lengthInverse);
}

intFix vec3fAng(struct vec3f * vecA, struct vec3f * vecB)
{	
	vec3fUnitVecEnhance(vecA);
	vec3fUnitVecEnhance(vecB);
	return fixAcos(vec3fDotProd(vecA, vecB));
}

void mat3fSend(struct mat3f * mat)
{
	Serial.write((const char*)&mat->data, 9 * sizeof(intFix));
	Serial.write('\n');
}

void mat3fPrint(struct mat3f * mat)
{
	Serial.printf("%f %f %f %f %f %f %f %f %f ",
	    fix2Float(mat->data[0][0]), fix2Float(mat->data[0][1]), fix2Float(mat->data[0][2]),
        fix2Float(mat->data[1][0]), fix2Float(mat->data[1][1]), fix2Float(mat->data[1][2]),
		fix2Float(mat->data[2][0]), fix2Float(mat->data[2][1]), fix2Float(mat->data[2][2]));
}

void mat3fZero(struct mat3f * mat)
{
	for (int i = 0; i < 9; i++)
		((intFix*)mat->data)[i] = 0;
}

void mat3fEye(struct mat3f * mat)
{
	mat3fZero(mat);
	mat->data[0][0] = FIX_UNITY;
	mat->data[1][1] = FIX_UNITY;
	mat->data[2][2] = FIX_UNITY;
}

void mat3fMult(struct mat3f * matA, struct mat3f * matB, struct mat3f * matOut)
{	
	struct mat3f matTmp;
	mat3fZero(&matTmp);
	
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			for (int k = 0; k < 3; k++)
				matTmp.data[i][j] += fixMult(matA->data[i][k], matB->data[k][j]);
	*matOut = matTmp;
}

void mat3fMultVec(struct mat3f * mat, struct vec3f * vecIn, struct vec3f * vecOut)
{
	struct vec3f vecTmp;
	vec3fZero(&vecTmp);
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			vecTmp.data[i] += fixMult(mat->data[i][j], vecIn->data[j]);
	*vecOut = vecTmp;		
}

void mat3fTranspose(struct mat3f * matIn, struct mat3f * matOut)
{	
	struct mat3f matTmp = *matIn;
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			matTmp.data[i][j] = matIn->data[j][i];
	*matOut = matTmp;
}

void mat3fExtractColumn(struct mat3f * mat, struct vec3f * vec, int col)
{
	vec->data[0] = mat->data[0][col];
	vec->data[1] = mat->data[1][col];
	vec->data[2] = mat->data[2][col];
}

void mat3fExtractRow(struct mat3f * mat, struct vec3f * vec, int row)
{
	vec->data[0] = mat->data[row][0];
	vec->data[1] = mat->data[row][1];
	vec->data[2] = mat->data[row][2];
}

void mat3fSetColumn(struct vec3f * vec, struct mat3f * mat, int col)
{
	mat->data[0][col] = vec->data[0];
	mat->data[1][col] = vec->data[1];
	mat->data[2][col] = vec->data[2];
}

void mat3fSetRowMan(int row, struct mat3f * mat, intFix a, intFix b, intFix c)
{
	mat->data[row][0] = a;
	mat->data[row][1] = b;
	mat->data[row][2] = c;
}

intFix mat3fDet(struct mat3f * mat)
{
	intFix result = 0;
	result += fixMult3(mat->data[0][0], mat->data[1][1], mat->data[2][2]);
	result += fixMult3(mat->data[0][1], mat->data[1][2], mat->data[2][0]);
	result += fixMult3(mat->data[0][2], mat->data[1][0], mat->data[2][1]);
	result -= fixMult3(mat->data[0][1], mat->data[1][0], mat->data[2][2]);
	result -= fixMult3(mat->data[0][0], mat->data[1][2], mat->data[2][1]);
	result -= fixMult3(mat->data[0][2], mat->data[1][1], mat->data[2][0]);
	return result;
}

void mat3fRotAxes(struct mat3f * mat, intFix theta, int axisA, int axisB)
{
	struct vec3f vecA;
	struct vec3f vecB;
	mat3fExtractColumn(mat, &vecA, axisA);
	mat3fExtractColumn(mat, &vecB, axisB);

	struct vec3f vecTmp;
	intFix cosTheta = fixCos(theta);
	intFix sinTheta = fixSin(theta);

	vec3fZero(&vecTmp);
	vec3fAccumMult(&vecTmp, &vecA, cosTheta);
	vec3fAccumMult(&vecTmp, &vecB, sinTheta);
	mat3fSetColumn(&vecTmp, mat, axisA);

	vec3fZero(&vecTmp);
	vec3fAccumMult(&vecTmp, &vecA, -sinTheta);
	vec3fAccumMult(&vecTmp, &vecB,  cosTheta);
	mat3fSetColumn(&vecTmp, mat, axisB);
}

void mat3fGyrRot(struct vec3f * vecAng, struct mat3f * matOri)
{	
	mat3fRotAxes(matOri, vecAng->data[0], 1, 2);
	mat3fRotAxes(matOri, vecAng->data[1], 2, 0);
	mat3fRotAxes(matOri, vecAng->data[2], 0, 1);
}

void mat3fRotFromAxis(struct vec3f * vecAxis, struct mat3f * matRot, intFix theta)
{	
	intFix cosTheta = fixCos(theta);
	intFix sinTheta = fixSin(theta);
	intFix oneMinusCos = FIX_UNITY - cosTheta;

	matRot->data[0][0] = cosTheta + fixMult3(vecAxis->data[0], vecAxis->data[0], oneMinusCos);
	matRot->data[0][1] = fixMult3(vecAxis->data[0], vecAxis->data[1], oneMinusCos) - fixMult(vecAxis->data[2], sinTheta);
	matRot->data[0][2] = fixMult3(vecAxis->data[0], vecAxis->data[2], oneMinusCos) + fixMult(vecAxis->data[1], sinTheta);
	matRot->data[1][0] = fixMult3(vecAxis->data[0], vecAxis->data[1], oneMinusCos) + fixMult(vecAxis->data[2], sinTheta);
	matRot->data[1][1] = cosTheta + fixMult3(vecAxis->data[1], vecAxis->data[1], oneMinusCos);
	matRot->data[1][2] = fixMult3(vecAxis->data[1], vecAxis->data[2], oneMinusCos) - fixMult(vecAxis->data[0], sinTheta);
	matRot->data[2][0] = fixMult3(vecAxis->data[0], vecAxis->data[2], oneMinusCos) - fixMult(vecAxis->data[1], sinTheta);
	matRot->data[2][1] = fixMult3(vecAxis->data[1], vecAxis->data[2], oneMinusCos) + fixMult(vecAxis->data[0], sinTheta);
	matRot->data[2][2] = cosTheta + fixMult3(vecAxis->data[2], vecAxis->data[2], oneMinusCos);
}

void mat3fRotFromVecPair(struct vec3f * vecA, struct vec3f * vecB, struct mat3f * matRot, intFix theta)
{
	struct vec3f vecAxis;
	vec3fCrossProd(vecA, vecB, &vecAxis);
	vec3fNorm(&vecAxis);
	vec3fUnitVecEnhance(&vecAxis);
	mat3fRotFromAxis(&vecAxis, matRot, theta);
}

void mat3fAccAlign(struct vec3f * vecAcc, struct mat3f * matOri, intFix weight, intFix max)
{		
		struct vec3f vecZ;
		struct vec3f vecTmp;
		mat3fExtractRow(matOri, &vecZ, 2);
		vecTmp = *vecAcc;
		vec3fNorm(&vecTmp);

		intFix theta = vec3fAng(&vecZ, &vecTmp);
		theta = limit(fixMult(theta, weight), -max, max); 
		
		struct mat3f matRotAlign;
		mat3fRotFromVecPair(&vecZ, &vecTmp, &matRotAlign, -theta); // Negation can be removed by switching inputs to crossproduct
		mat3fMult(matOri, &matRotAlign, matOri);
}

void mat3fMagAlign(struct vec3f * vecMag, struct mat3f * matOri, intFix weight, intFix max)
{	
	struct vec3f vecNorth;
	mat3fMultVec(matOri, vecMag, &vecNorth);

	intFix theta = fixAtan2(vecNorth.data[1], vecNorth.data[0]);
	theta = limit(fixMult(theta, weight), -max, max);

	struct mat3f matRotAlign;
	mat3fEye(&matRotAlign);
	mat3fRotAxes(&matRotAlign, -theta, 0, 1);
	mat3fMult(&matRotAlign, matOri, matOri);
}

void mat3fOrthoFix(struct mat3f * mat)
{
	struct vec3f vecOldX;
	struct vec3f vecOldY;
	
	mat3fExtractColumn(mat, &vecOldX, 0);
	mat3fExtractColumn(mat, &vecOldY, 1);
	
	intFix err = vec3fDotProd(&vecOldX, &vecOldY);
	struct vec3f vecX = vecOldY;
	vec3fMult(&vecX, (-0.5) * err);
	vec3fAccum(&vecX, &vecOldX);
	vec3fNorm(&vecX);
	mat3fSetColumn(&vecX, mat, 0);
	
	struct vec3f vecY = vecOldX;
	vec3fMult(&vecY, (-0.5) * err);
	vec3fAccum(&vecY, &vecOldY);
	vec3fNorm(&vecY);
	mat3fSetColumn(&vecY, mat, 1);
	
	struct vec3f vecZ;
	vec3fCrossProd(&vecX, &vecY, &vecZ);
	mat3fSetColumn(&vecZ, mat, 2);
}

