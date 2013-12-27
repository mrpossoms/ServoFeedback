#include "feedbacklib.h"

float dt = 0.016f; // TODO get real dt
const float VAR_BIAS = 0.000001f;

float __filter_update(ServoFiltering* f, float m){
    // Predict to now, then update.
    // Predict:
    //   X = F*X + H*U
    //   P = F*X*F^T + Q.
    // Update:
    //   Y = M – H*X          Called the innovation = measurement – state transformed by H.	
    //   S = H*P*H^T + R      S= Residual covariance = covariane transformed by H + R
    //   K = P * H^T *S^-1    K = Kalman gain = variance / residual covariance.
    //   X = X + K*Y          Update with gain the new measurement
    //   P = (I – K * H) * P  Update covariance to this time.

    // X = F*X + H*U
    float oldX = f->m_x[0];
    f->m_x[0] = f->m_x[0] + (dt * f->m_x[1]);

    // P = F*X*F^T + Q
    f->m_p[0] = f->m_p[0] + dt * (f->m_p[2] + f->m_p[1]) + dt * dt * f->m_p[3] + f->m_q[0];
    f->m_p[1] = f->m_p[1] + dt * f->m_p[3] + f->m_q[1];
    f->m_p[2] = f->m_p[2] + dt * f->m_p[3] + f->m_q[2];
    f->m_p[3] = f->m_p[3] + f->m_q[3];

    // Y = M – H*X
    {
    	int i = 4;
	    float y0 = m - f->m_x[0];
	    float y1 = ((m - oldX) / dt) - f->m_x[1];

	    // S = H*P*H^T + R 
	    // Because H = [1, 0] this is easy, and s is a single value not a matrix to invert.
	    float s = f->m_p[0] + f->m_r;

	    // K = P * H^T *S^-1 
	    float k = f->m_p[0] / s;
	    f->LastGain = k;

	    // X = X + K*Y
	    f->m_x[0] += y0 * k;
	    f->m_x[1] += y1 * k;

	    // P = (I – K * H) * P
	    for (;i--;) f->m_p[i] = f->m_p[i] - k * f->m_p[i];
	}

	// Return latest estimate.
	return f->m_x[0];
}
//------------------------------------------------------------------------
Servo fbGenServo(void (*impactCB)(Servo*, float), float angle, float varCoeff){

	// instantiate stats object
	ServoStats stats = {
		{0}, // samples
		varCoeff,
		0
	};

	// instantiate the filtering object
	float qx = 0.5f, qv = 0.5f;
	float pd = 0.5f;
	ServoFiltering filter = {
		{0}, // samples
		{angle, 0},
		{pd, 0, 0, pd},
		{qx, qv, 0, 0},
		1.0f,
		0
	};

	// instantiate the servo object
	Servo servo = {
		angle, // current angle
		angle, // last angle
		stats,
		filter,
		0,     // current sample index
		impactCB
	};

	return servo;
}
//------------------------------------------------------------------------
void fbSetAngle(Servo* servo, float angle){
	servo->LastAngle = servo->Angle;
	servo->Angle = angle;
}
//------------------------------------------------------------------------
void fbUpdate(Servo* servo, float sample){
	int i, sampInd = servo->CurrentIndex;
	ServoStats* stats = &servo->stats;
	ServoFiltering* filtr = &servo->filter;
	float impactVar = stats->LastVar * stats->MaxVarCoeff;
	float var = 0;

	// store sample values
	stats->Samples[sampInd] = sample;
	filtr->Samples[sampInd] = __filter_update(
		&servo->filter,
		sample
	);

	// determine variance of raw feedback signal
	for(i = 0; i < RAW_SAMPLE_COUNT; i++){
		float raw = stats->Samples[i];
		float flt = filtr->Samples[i];

		register float term = raw - flt;
		var += term * term;
	}
	var *= 1/((float)RAW_SAMPLE_COUNT);

	// check to see if an impact likely occured
	if(stats->LastVar > VAR_BIAS && var > impactVar){
		if(servo->impactCallback)
		servo->impactCallback(servo, var); // impact, fire callback
	}

	// Keep the most recently calculated variance
	stats->LastVar = var;

	// move to the next sample index
	servo->CurrentIndex = (sampInd + 1) % RAW_SAMPLE_COUNT;
}
