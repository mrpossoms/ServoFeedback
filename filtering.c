#include "filtering.h"

void Reset(KalmanFilter* f, double qx, double qv, double r, double pd, double ix){
	f->m_q[0] = qx; f->m_q[1] = qv;
	f->m_r = r;
	f->m_p[0] = f->m_p[3] = pd; 
	f->m_p[1] = f->m_p[2] = 0;
	f->m_x[0] = ix; 
	f->m_x[1] = 0;
}

double Update(KalmanFilter* f, double m, double dt){
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
    double oldX = f->m_x[0];
    f->m_x[0] = f->m_x[0] + (dt * f->m_x[1]);

    // P = F*X*F^T + Q
    f->m_p[0] = f->m_p[0] + dt * (f->m_p[2] + f->m_p[1]) + dt * dt * f->m_p[3] + f->m_q[0];
    f->m_p[1] = f->m_p[1] + dt * f->m_p[3] + f->m_q[1];
    f->m_p[2] = f->m_p[2] + dt * f->m_p[3] + f->m_q[2];
    f->m_p[3] = f->m_p[3] + f->m_q[3];

    // Y = M – H*X
    {
    	int i = 4;
	    double y0 = m - f->m_x[0];
	    double y1 = ((m - oldX) / dt) - f->m_x[1];

	    // S = H*P*H^T + R 
	    // Because H = [1, 0] this is easy, and s is a single value not a matrix to invert.
	    double s = f->m_p[0] + f->m_r;

	    // K = P * H^T *S^-1 
	    double k = f->m_p[0] / s;
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