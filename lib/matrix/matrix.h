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
void mat3Zero(struct mat3 * mat);
void mat3Eyes(struct mat3 * mat);
void mat3SetRow(int row, struct mat3 * mat, float a, float b, float c);
void mat3Cpy(struct mat3 * matSource, struct mat3 * matTarget);
void mat3Mult(struct mat3 * matA, struct mat3 * matB, struct mat3 * matC);
void mat3RotByGyr(struct vec3 * vec, struct mat3 * matOris, float timeElapsed);