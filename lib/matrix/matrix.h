#include <math.h>

struct mat3 {
	float data[3][3];
};

struct vec3 {
	float data[3];
};

void mat3Print(struct mat3 * mat);
void vec3Print(struct vec3 * vec);
void vec3Mult(struct vec3 * vec, float k);
void vec3Zero(struct vec3 * vec);
void vec3Set(struct vec3 * vec, float x, float y, float z);
void mat3Zero(struct mat3 * mat);
void mat3Eyes(struct mat3 * mat);
void mat3SetRow(int row, struct mat3 * mat, float a, float b, float c);
void mat3Mult(struct mat3 * matA, struct mat3 * matB, struct mat3 * matOut);
void mat3ExtractColumn(struct mat3 * mat, struct vec3 * vec, int col);
void mat3ExtractRow(struct mat3 * mat, struct vec3 * vec, int row);
void mat3SetColumn(struct vec3 * vec, struct mat3 * mat, int col);
void mat3RotFromAxis(struct vec3 * vecAxis, struct mat3 * matRot, float theta); // Axis vector must be normalized
void mat3GyrRot(struct vec3 * vecVel, struct mat3 * matOri, float timeElapsed);
void mat3AccAlign(struct vec3 * vecAcc, struct mat3 * matOri, float weight, float max);
void mat3MagAlign(struct vec3 * vecMag, struct mat3 * matOri, float weight, float max);
float vec3DotProd(struct vec3 * vecA, struct vec3 * vecB);
void vec3CrossProd(struct vec3 * vecA, struct vec3 * vecB, struct vec3 * vecCross);
void vec3Add(struct vec3 * vecA, struct vec3 * vecB, struct vec3 * vecOut);
float vec3Length(struct vec3 * vec);
void vec3Norm(struct vec3 * vec);
void mat3OrthoFix(struct mat3 * mat);
void mat3MultVec(struct mat3 * mat, struct vec3 * vecIn, struct vec3 * vecOut);
void mat3Transpose(struct mat3 * matIn, struct mat3 * matOut);
float vec3GetAng(struct vec3 * vecA, struct vec3 * vecB); // Vectors must be normalized
void mat3RotFromVecPair(struct vec3 * vecA, struct vec3 * vecB, struct mat3 * matRot, float theta);
void mat3RotZ(struct mat3 * matRot, float theta);
float mat3Det(struct mat3 * mat);
void vec3Accum(struct vec3 * vecAccum, struct vec3 * vecAdd);
