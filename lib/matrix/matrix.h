#include <math.h>

struct mat3
{
	float data[3][3];
};

struct vec3
{
	float data[3];
};

void mat3Print(struct mat3 * mat);
void vec3Print(struct vec3 * vec);
void vec3Zero(struct vec3 * vec);
void vec3MultFac(struct vec3 * vec, float k);
void vec3Zero(struct vec3 * vec);
void mat3Zero(struct mat3 * mat);
void mat3Eyes(struct mat3 * mat);
void mat3SetRow(int row, struct mat3 * mat, float a, float b, float c);
void mat3Mult(struct mat3 * matA, struct mat3 * matB, struct mat3 * matOut);
void mat3ExtractColumn(struct mat3 * mat, struct vec3 * vec, int col);
void mat3SetColumn(struct vec3 * vec, struct mat3 * mat, int col);
void mat3RotFromAxis(struct vec3 * vecAxis, struct mat3 * matRot, float theta);
void mat3RotByGyr(struct vec3 * vecVel, struct mat3 * matOri, float timeElapsed);
float vec3DotProd(struct vec3 * vecA, struct vec3 * vecB);
void vec3CrossProd(struct vec3 * vecA, struct vec3 * vecB, struct vec3 * vecCross);
void vec3Add(struct vec3 * vecA, struct vec3 * vecB, struct vec3 * vecOut);
void vec3Norm(struct vec3 * vec);
void mat3OrthoFix(struct mat3 * mat);