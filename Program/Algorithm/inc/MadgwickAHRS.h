//=====================================================================================================
// MadgwickAHRS.h
//=====================================================================================================
//
// Implementation of Madgwick's IMU and AHRS algorithms.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
// Date			Author          Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
//
//=====================================================================================================
#ifndef MadgwickAHRS_h
#define MadgwickAHRS_h

//----------------------------------------------------------------------------------------------------
// Variable declaration


extern volatile double q0, q1, q2, q3;	// quaternion of sensor frame relative to auxiliary frame


extern volatile double twoKp;			// 2 * proportional gain (Kp)
extern volatile double twoKi;			// 2 * integral gain (Ki)
extern volatile double q0, q1, q2, q3;	// quaternion of sensor frame relative to auxiliary frame

//---------------------------------------------------------------------------------------------------
// Function declarations


void MahonyAHRSupdate(double gx, double gy, double gz, double ax, double ay, double az, double mx, double my, double mz,double *q);
void MahonyAHRSupdateIMU(double gx, double gy, double gz, double ax, double ay, double az,double *q);
//---------------------------------------------------------------------------------------------------
// Function declarations

void MadgwickAHRSupdate(double gx, double gy, double gz, double ax, double ay, double az, double mx, double my, double mz,double *q) ;
void MadgwickAHRSupdateIMU(double gx, double gy, double gz, double ax, double ay, double az,double *q);
void Quaternion_ToEuler(float *q, float* rpy);
#endif
//=====================================================================================================
// End of file
//=====================================================================================================
