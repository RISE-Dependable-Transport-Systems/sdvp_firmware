#ifndef POS_H_
#define POS_H_

#include "datatypes.h"

// Functions
void pos_init(void);
void pos_imu_data_callback(float *accel, float *gyro, float *mag);
void pos_get_pos(POS_STATE *p);
void pos_get_imu(float *accel, float *gyro, float *mag);

#endif /* POS_H_ */
