#include <math.h>

struct mat3 {
	float data[3][3];
};

struct vec3 {
	float data[3];
};

void vec3Print(struct vec3 * vec);
void vec3Zero(struct vec3 * vec);
void vec3Set(struct vec3 * vec, float x, float y, float z);
void vec3Mult(struct vec3 * vec, float k);
void vec3Add(struct vec3 * vecA, struct vec3 * vecB, struct vec3 * vecOut);
void vec3Accum(struct vec3 * vecAccum, struct vec3 * vecAdd);
void vec3AccumMult(struct vec3 * vecAccum, struct vec3 * vecAdd, float k);
float vec3DotProd(struct vec3 * vecA, struct vec3 * vecB);
void vec3CrossProd(struct vec3 * vecA, struct vec3 * vecB, struct vec3 * vecCross);
float vec3Length(struct vec3 * vec);
void vec3Norm(struct vec3 * vec);
float vec3GetAng(struct vec3 * vecA, struct vec3 * vecB);
void mat3Print(struct mat3 * mat);
void mat3Zero(struct mat3 * mat);
void mat3Eyes(struct mat3 * mat);
void mat3Mult(struct mat3 * matA, struct mat3 * matB, struct mat3 * matOut);
void mat3MultVec(struct mat3 * mat, struct vec3 * vecIn, struct vec3 * vecOut);
void mat3Transpose(struct mat3 * matIn, struct mat3 * matOut);
void mat3ExtractColumn(struct mat3 * mat, struct vec3 * vec, int col);
void mat3ExtractRow(struct mat3 * mat, struct vec3 * vec, int row);
void mat3SetColumn(struct vec3 * vec, struct mat3 * mat, int col);
void mat3SetRowMan(int row, struct mat3 * mat, float a, float b, float c);
float mat3Det(struct mat3 * mat);
void mat3RotAxisX(struct mat3 * mat, float theta);
void mat3RotAxisY(struct mat3 * mat, float theta);
void mat3RotAxisZ(struct mat3 * mat, float theta);
void mat3GyrRot(struct vec3 * vecVel, struct mat3 * matOri, float timeElapsed);
void mat3RotFromAxis(struct vec3 * vecAxis, struct mat3 * matRot, float theta);
void mat3RotFromVecPair(struct vec3 * vecA, struct vec3 * vecB, struct mat3 * matRot, float theta);
void mat3AccAlign(struct vec3 * vecAcc, struct mat3 * matOri, float weight, float max);
void mat3RotZ(struct mat3 * matRot, float theta);
void mat3MagAlign(struct vec3 * vecMag, struct mat3 * matOri, float weight, float max);
void mat3OrthoFix(struct mat3 * mat);