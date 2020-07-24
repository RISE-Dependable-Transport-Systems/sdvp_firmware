#ifndef POS_GNSS_H_
#define POS_GNSS_H_

#include "datatypes.h"

// Functions
void pos_gnss_init(void);
void pos_gnss_get(GPS_STATE *p);
void pos_gnss_set_enu_ref(double lat, double lon, double height);
void pos_gnss_get_enu_ref(double *llh);
void pos_gnss_nmea_cb(const char *data);

#endif /* POS_GNSS_H_ */
