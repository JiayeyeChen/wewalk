#include "imu_task.h"
#include "bmi088driver.h"
#include "MahonyAHRS.h"
#include <math.h>

#define DES_TEMP    40.0f
#define KP          100.f
#define KI          50.f
#define KD          10.f
#define MAX_OUT     500

float imuQuat[4] = {0.0f};
float imuAngle[3] = {0.0f};
float imuAngleDeg[3] = {0.0f};

float out = 0;
float err = 0;
float err_l = 0;
float err_ll = 0;

void AHRS_init(float quat[4])
{
    quat[0] = 1.0f;
    quat[1] = 0.0f;
    quat[2] = 0.0f;
    quat[3] = 0.0f;

}

void AHRS_update(float quat[4], float gyro[3], float accel[3])
{
    MahonyAHRSupdateIMU(quat, gyro[0], gyro[1], gyro[2], accel[0], accel[1], accel[2]);
}

void GetAngle(float q[4], float *yaw, float *pitch, float *roll)
{
    *yaw = atan2f(2.0f*(q[0]*q[3]+q[1]*q[2]), 2.0f*(q[0]*q[0]+q[1]*q[1])-1.0f);
    *pitch = asinf(-2.0f*(q[1]*q[3]-q[0]*q[2]));
    *roll = atan2f(2.0f*(q[0]*q[1]+q[2]*q[3]),2.0f*(q[0]*q[0]+q[3]*q[3])-1.0f);
}


void ImuTask(float gyro[3], float acc[3], float temp)
{
  AHRS_update(imuQuat, gyro, acc);
  GetAngle(imuQuat, imuAngle + INS_YAW_ADDRESS_OFFSET, imuAngle + INS_PITCH_ADDRESS_OFFSET, imuAngle + INS_ROLL_ADDRESS_OFFSET);
  err_ll = err_l;
  err_l = err;
  err = DES_TEMP - temp;
  out = KP*err + KI*(err + err_l + err_ll) + KD*(err - err_l);
  if (out > MAX_OUT) out = MAX_OUT;
  if (out < 0) out = 0.f;
  
  imuAngleDeg[0] = imuAngle[2] * 57.29577951f;
  imuAngleDeg[1] = imuAngle[1] * 57.29577951f;
  imuAngleDeg[2] = imuAngle[0] * 57.29577951f;

}



