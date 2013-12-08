#ifndef FILTERING
#define FILTERING

#include <math.h>

#define e 2.718281828f
#define SQRT_2PI 2.506628275f
#define S 0.4f
#define TWO_SSQR 2 * S * S

typedef struct{
        double m_x[2];
        /// Covariance.
        double m_p[4];
        /// Minimal covariance.
        double m_q[4];
        /// Minimal innovative covariance, keeps filter from locking in to a solution.
        double m_r;

        double LastGain;
} KalmanFilter;

void Reset(KalmanFilter* f, double qx, double qv, double r, double pd, double ix);
double Update(KalmanFilter* f, double m, double dt);

#endif