#ifndef POS_H_
#define POS_H_

// Functions
void pos_init(void);
void pos_imu_data_callback(float *accel, float *gyro, float *mag);
void pos_get_imu(float *accel, float *gyro, float *mag);

#endif /* POS_H_ */
