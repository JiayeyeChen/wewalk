#ifndef __IMU_TASK_H__
#define __IMU_TASK_H__

#define INS_YAW_ADDRESS_OFFSET    0
#define INS_PITCH_ADDRESS_OFFSET  1
#define INS_ROLL_ADDRESS_OFFSET   2


void ImuTask(float gyro[3], float acc[3], float temp);
void AHRS_init(float quat[4]);
extern float imuQuat[4];
extern float imuAngle[3];
extern float imuAngleDeg[3];
#endif
