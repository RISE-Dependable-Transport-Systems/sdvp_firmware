#ifndef POS_H_
#define POS_H_

#include "datatypes.h"

// Functions
void pos_init(void);
void pos_imu_data_callback(float *accel, float *gyro, float *mag);
void pos_get_imu(float *accel, float *gyro, float *mag);
void pos_get_pos(POS_STATE *p);
float pos_get_yaw(void);
float pos_get_speed(void);
void pos_set_xya(float x, float y, float angle);
void pos_set_yaw_offset(float angle);
void cmd_terminal_reset_attitude(int argc, const char **argv);
void pos_correction_gnss(const float gnss_px, const float gnss_py, const float gnss_pz, const int32_t gnss_ms, const int fix_type);
void pos_mc_values_received(mc_values *val);

#endif /* POS_H_ */
