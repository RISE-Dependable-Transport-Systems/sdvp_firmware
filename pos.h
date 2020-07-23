#ifndef POS_H_
#define POS_H_

#include "datatypes.h"

// Functions
void pos_init(void);
void pos_imu_data_callback(float *accel, float *gyro, float *mag);
void pos_get_imu(float *accel, float *gyro, float *mag);
void pos_get_pos(POS_STATE *p);
void pos_get_gps(GPS_STATE *p);
float pos_get_speed(void);
void pos_set_xya(float x, float y, float angle);
void pos_set_yaw_offset(float angle);
void pos_set_enu_ref(double lat, double lon, double height);
void pos_get_enu_ref(double *llh);
void cmd_terminal_reset_enu_ref(int argc, const char **argv);
void pos_input_nmea(const char *data);
void cmd_terminal_reset_attitude(int argc, const char **argv);
void pos_mc_values_received(mc_values *val);

#endif /* POS_H_ */
