#ifndef __FEEDBACK_LIB
#define __FEEDBACK_LIB

#include "./../arduino-serial-lib.h"

#define RAW_SAMPLE_COUNT 100
#define e 2.718281828f
#define SQRT_2PI 2.506628275f
#define S 0.4f
#define TWO_SSQR 2 * S * S

typedef struct{
	// Circular feedback samples
	float Samples[RAW_SAMPLE_COUNT];

	// Statistical affectors
	float MaxVarCoeff; // Variance delta that cannot be exceeded
	float LastVar;     // Last variance
} ServoStats;
//------------------------------------------------------------------------
typedef struct{
	// kalman filtering stuff
	float Samples[RAW_SAMPLE_COUNT];
	float m_x[2]; // State vector
	float m_p[4]; // Covariance
	float m_q[4]; // Minimal covariance
	float m_r;    // Minimal innovative covariance
	float LastGain;
} ServoFiltering;
//------------------------------------------------------------------------
struct __servo;
struct __servo{
	// Target angle, and the previous target
	float Angle, LastAngle;

	// Statistical affectors
	ServoStats stats;

	// kalman filtering stuff
	ServoFiltering filter;

	// Most current sample index
	int CurrentIndex;

	// function pointer to behavior invoked when
	// impacting an obstruction
	void (*impactCallback)(struct __servo* servo, float);
};
typedef struct __servo Servo;
//------------------------------------------------------------------------

Servo fbGenServo(void (*impactCB)(Servo*, float), float angle, float varCoeff);
void  fbSetAngle(Servo* servo, float angle);
void  fbUpdate(Servo* servo, float sample);

#endif
