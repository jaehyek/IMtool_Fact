



void EKF_Init(EKF_Filter* ekf, float32_t *q, float32_t *gyro)
{	
	float32_t *X = ekf->X_f32;
	float32_t norm;

	X[0] = q[0];
	X[1] = q[1];
	X[2] = q[2];
	X[3] = q[3];

	norm = FastSqrtI(X[0] * X[0] + X[1] * X[1] + X[2] * X[2] + X[3] * X[3]);
	X[0] *= norm;
	X[1] *= norm;
	X[2] *= norm;
	X[3] *= norm;

	X[4] = gyro[0];
	X[5] = gyro[1];
	X[6] = gyro[2];
}