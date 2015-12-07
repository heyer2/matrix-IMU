#ifndef MATRIXFIX_H
#define MATRIXFIX_H

#include <fixmath.h>

struct mat3f {
	intFix data[3][3];
};

struct vec3f {
	intFix data[3];
};

void vec3fSend(struct vec3f * vec);
void vec3fPrint(struct vec3f * vec);
void vec3fZero(struct vec3f * vec);
void vec3fSet(struct vec3f * vec, intFix x, intFix y, intFix z);
void vec3fDiv(struct vec3f * vec, intFix k);
void vec3fDivVec(struct vec3f * vecRes, struct vec3f * vec);
void vec3fMult(struct vec3f * vec, intFix k);
void vec3fMultVec(struct vec3f * vecRes, struct vec3f * vec);
void vec3fAdd(struct vec3f * vecA, struct vec3f * vecB, struct vec3f * vecOut);
void vec3fAccum(struct vec3f * vecAccum, struct vec3f * vecAdd);
void vec3fAccumMult(struct vec3f * vecAccum, struct vec3f * vecAdd, intFix k);
longFix vec3fDotProd(struct vec3f * vecA, struct vec3f * vecB);
void vec3fCrossProd(struct vec3f * vecA, struct vec3f * vecB, struct vec3f * vecCross);
intFix vec3fLength(struct vec3f * vec);
void vec3fNorm(struct vec3f * vec);
intFix vec3fAng(struct vec3f * vecA, struct vec3f * vecB);
void mat3fSend(struct mat3f * mat);
void mat3fPrint(struct mat3f * mat);
void mat3fZero(struct mat3f * mat);
void mat3fEye(struct mat3f * mat);
void mat3fMult(struct mat3f * matA, struct mat3f * matB, struct mat3f * matOut);
void mat3fMultVec(struct mat3f * mat, struct vec3f * vecIn, struct vec3f * vecOut);
void mat3fTranspose(struct mat3f * matIn, struct mat3f * matOut);
void mat3fExtractColumn(struct mat3f * mat, struct vec3f * vec, int col);
void mat3fExtractRow(struct mat3f * mat, struct vec3f * vec, int row);
void mat3fSetColumn(struct vec3f * vec, struct mat3f * mat, int col);
void mat3fSetRowMan(int row, struct mat3f * mat, intFix a, intFix b, intFix c);
intFix mat3fDet(struct mat3f * mat);
void mat3fRotAxes(struct mat3f * mat, intFix theta, int axisA, int axisB);
void mat3fGyrRot(struct vec3f * vecAng, struct mat3f * matOri);
void mat3fRotFromAxis(struct vec3f * vecAxis, struct mat3f * matRot, intFix theta);
void mat3fRotFromVecPair(struct vec3f * vecA, struct vec3f * vecB, struct mat3f * matRot, intFix theta);
void mat3fAccAlign(struct vec3f * vecAcc, struct mat3f * matOri, intFix weight, intFix max);
void mat3fMagAlign(struct vec3f * vecMag, struct mat3f * matOri, intFix weight, intFix max);
void mat3fOrthoFix(struct mat3f * mat);

#endif /* MATRIXFIX_H */