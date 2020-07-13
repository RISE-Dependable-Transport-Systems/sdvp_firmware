#ifndef CONF_GENERAL_H_
#define CONF_GENERAL_H_

// General settings
#define ID_ALL						255
#define ID_CAR_CLIENT				254 // Packet for car client only
#ifndef VESC_ID
#define VESC_ID						ID_ALL // id, or ID_ALL for any VESC (not used in diff steering mode)
#endif

// Global variables
extern int main_id;

#endif /* CONF_GENERAL_H_ */
