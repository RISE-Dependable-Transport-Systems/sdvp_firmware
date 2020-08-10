#ifndef POS_H_
#define POS_H_

#include "datatypes.h"

// Functions
void pos_init(void);
void pos_get_pos(POS_STATE *p);
float pos_get_yaw(void);
float pos_get_speed(void);
void pos_set_xya(float x, float y, float angle);
void pos_set_yaw_offset(float angle);
void pos_correction_imu(const float roll, const float pitch, const float yaw, const float yaw_mag, const float gyro[3], const float quaternions[4], const float dt);
void pos_correction_gnss(const float gnss_px, const float gnss_py, const float gnss_pz, const int32_t gnss_ms, const int fix_type);
void pos_correction_mc(float distance, float turn_rad_rear, float angle_diff, float speed);

#endif /* POS_H_ */
