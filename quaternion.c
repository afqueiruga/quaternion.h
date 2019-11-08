#include "quaternion.h"

#include <math.h>

void quat_conj( real * q, real * qc ) {
  qc[0] =  q[0];
  qc[1] = -q[1];
  qc[2] = -q[2];
  qc[3] = -q[3];
}

void quat_mul(real * q, real * p, real * r) {
  r[0] = q[0]*p[0] - q[1]*p[1] - q[2]*p[2] - q[3]*p[3];
  r[1] = q[0]*p[1] + q[1]*p[0] + q[2]*p[3] - q[3]*p[2];
  r[2] = q[0]*p[2] - q[1]*p[3] + q[2]*p[1] + q[3]*p[1];
  r[3] = q[0]*p[3] + q[1]*p[2] - q[2]*p[2] + q[3]*p[0];
}

void quat_rot(real * q, real * v, real * Rv) {
  Rv[0] = q[0]*q[0]*v[0] + q[1]*q[1]*v[0] - (q[2]*q[2] + q[3]*q[3])*v[0] + q[0]*(-2.0*q[3]*v[1] + 2.0*q[2]*v[2]) + 2.0*q[1]*(q[2]*v[1] + q[3]*v[2]);
  Rv[1] = 2.0*q[1]*q[2]*v[0] + 2.0*q[0]*q[3]*v[0] + q[0]*q[0]*v[1] - q[1]*q[1]*v[1] + q[2]*q[2]*v[1] - q[3]*q[3]*v[1] - 2.0*q[0]*q[1]*v[2] + 2.0*q[2]*q[3]*v[2];
  Rv[2] = q[0]*(-2.0*q[2]*v[0] + 2.0*q[1]*v[1]) + 2.0*q[3]*(q[1]*v[0] + q[2]*v[1]) + q[0]*q[0]*v[2] - (q[1]*q[1] + q[2]*q[2] - q[3]*q[3])*v[2];

}

void quat_make(real theta, real * u, real * q) {
  q[0] = cos(theta/2);
  q[1] = sin(theta/2)*u[0];
  q[2] = sin(theta/2)*u[1];
  q[3] = sin(theta/2)*u[2];
}

// This is for w in the global frame
void quat_dqdt_angular(real * q, real * w, real * dqdt) {
  dqdt[0] = (-(q[1]*w[0]) - q[2]*w[1] - q[3]*w[2])/2.0;
  dqdt[1] = (q[0]*w[0] + q[3]*w[1] - q[2]*w[2])/2.0;
  dqdt[2] = (-(q[3]*w[0]) + q[0]*w[1] + q[1]*w[2])/2.0;
  dqdt[3] = (q[2]*w[0] - q[1]*w[1] + q[0]*w[2])/2.0;
}

//This is for w in the local frame
void quat_dqdt_angular_local(real * q, real * w, real * dqdt) {
  dqdt[0] = (-(q[1]*w[0]) - q[2]*w[1] - q[3]*w[2])/2.0;
  dqdt[1] = (q[0]*w[0] - q[3]*w[1] + q[2]*w[2])/2.0;
  dqdt[2] = (q[3]*w[0] + q[0]*w[1] - q[1]*w[2])/2.0;
  dqdt[3] = (-(q[2]*w[0]) + q[1]*w[1] + q[0]*w[2])/2.0;
}

real quat_norm(real * q) {
  return q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3];
}

void quat_normalize(real * q) {
  real Lq = sqrt(quat_norm(q));
  for(int i=0;i<4;i++) q[i]/=Lq;
}
