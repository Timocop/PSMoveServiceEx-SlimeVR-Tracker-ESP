#ifndef _MADGWICK_H_
#define _MADGWICK_H_

#include "helper_3dmath.h"
void madgwickQuaternionUpdateStable(float q[4], float ax, float ay, float az, float gx, float gy, float gz, float deltat);
void madgwickQuaternionUpdateStable(float q[4], float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float deltat);
void madgwickQuaternionUpdate(float q[4], float ax, float ay, float az, float gx, float gy, float gz, float deltat);
void madgwickQuaternionUpdate(float q[4], float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float deltat);
void madgwickQuaternionUpdate(float q[4], float ax, float ay, float az, float gx, float gy, float gz, float deltat, float beta);
void madgwickQuaternionUpdate(float q[4], float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float deltat, float beta);

#endif /* _MADGWICK_H_ */