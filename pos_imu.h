#ifndef POS_IMU_H_
#define POS_IMU_H_

void pos_imu_init(void);
void pos_imu_data_cb(float *accel, float *gyro, float *mag);
void pos_imu_get(float *accel, float *gyro, float *mag);

#endif /* POS_IMU_H_ */
