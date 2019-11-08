#ifndef H_QUATERNION
#define H_QUATERNION

#ifndef real
#define real double
#endif

void quat_conj( real * q, real * qc );
void quat_mul(real * q, real * p, real * r);
void quat_rot(real * q, real * v, real * Rv);

void quat_make(real theta, real * u, real * q);
void quat_dqdt_angular(real * q, real * w, real * dqdt);

real quat_norm(real * q);
void quat_normalize(real * q);


#endif
