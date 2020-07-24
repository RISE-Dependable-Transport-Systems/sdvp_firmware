#include "pos_gnss.h"
#include "conf_general.h"
#include "utils.h"
#include "terminal.h"
#include "commands.h"
#include "time_today.h"
#include "pos.h"
#include "ch.h"
#include <math.h>
#include <stdlib.h>

// Private variables
static GPS_STATE m_gps;
static mutex_t m_mutex_gps;
static bool m_ubx_pos_valid;
static nmea_gsv_info_t m_gpgsv_last;
static nmea_gsv_info_t m_glgsv_last;

// Private functions
static void init_gps_local(GPS_STATE *gps);
static void cmd_terminal_reset_enu_ref(int argc, const char **argv);

void pos_gnss_init(void) {
	memset(&m_gps, 0, sizeof(m_gps));
	chMtxObjectInit(&m_mutex_gps);
	memset(&m_gpgsv_last, 0, sizeof(m_gpgsv_last));
	memset(&m_glgsv_last, 0, sizeof(m_glgsv_last));
	m_ubx_pos_valid = true;

	terminal_register_command_callback(
			"pos_reset_enu",
			"Re-initialize the ENU reference on the next GNSS sample",
			NULL,
			cmd_terminal_reset_enu_ref);
}

void pos_gnss_get(GPS_STATE *p) {
	chMtxLock(&m_mutex_gps);
	*p = m_gps;
	chMtxUnlock(&m_mutex_gps);
}

void pos_gnss_set_enu_ref(double lat, double lon, double height) {
	double x, y, z;
	utils_llh_to_xyz(lat, lon, height, &x, &y, &z);

	chMtxLock(&m_mutex_gps);

	m_gps.ix = x;
	m_gps.iy = y;
	m_gps.iz = z;

	float so = sinf((float)lon * M_PI / 180.0);
	float co = cosf((float)lon * M_PI / 180.0);
	float sa = sinf((float)lat * M_PI / 180.0);
	float ca = cosf((float)lat * M_PI / 180.0);

	// ENU
	m_gps.r1c1 = -so;
	m_gps.r1c2 = co;
	m_gps.r1c3 = 0.0;

	m_gps.r2c1 = -sa * co;
	m_gps.r2c2 = -sa * so;
	m_gps.r2c3 = ca;

	m_gps.r3c1 = ca * co;
	m_gps.r3c2 = ca * so;
	m_gps.r3c3 = sa;

	m_gps.lx = 0.0;
	m_gps.ly = 0.0;
	m_gps.lz = 0.0;

	m_gps.local_init_done = true;

	chMtxUnlock(&m_mutex_gps);
}

void pos_gnss_get_enu_ref(double *llh) {
	chMtxLock(&m_mutex_gps);
	utils_xyz_to_llh(m_gps.ix, m_gps.iy, m_gps.iz, &llh[0], &llh[1], &llh[2]);
	chMtxUnlock(&m_mutex_gps);
}


static void cmd_terminal_reset_enu_ref(int argc, const char **argv) {
	(void)argc;
	(void)argv;
	m_gps.local_init_done = false;
	terminal_printf("OK");
}

void pos_gnss_nmea_cb(const char *data) {
	nmea_gga_info_t gga;
	static nmea_gsv_info_t gpgsv;
	static nmea_gsv_info_t glgsv;
	int gga_res = utils_decode_nmea_gga(data, &gga);
	int gpgsv_res = utils_decode_nmea_gsv("GP", data, &gpgsv);
	int glgsv_res = utils_decode_nmea_gsv("GL", data, &glgsv);

	if (gpgsv_res == 1) {
		utils_sync_nmea_gsv_info(&m_gpgsv_last, &gpgsv);
	}

	if (glgsv_res == 1) {
		utils_sync_nmea_gsv_info(&m_glgsv_last, &glgsv);
	}

	if (gga.t_tow >= 0) {
		time_today_set_pps_time_ref(gga.t_tow);
	}

	// Only use valid fixes
	if (gga.fix_type == 1 || gga.fix_type == 2 || gga.fix_type == 4 || gga.fix_type == 5) {
		// Convert llh to ecef
		double sinp = sin(gga.lat * D_PI / D(180.0));
		double cosp = cos(gga.lat * D_PI / D(180.0));
		double sinl = sin(gga.lon * D_PI / D(180.0));
		double cosl = cos(gga.lon * D_PI / D(180.0));
		double e2 = FE_WGS84 * (D(2.0) - FE_WGS84);
		double v = RE_WGS84 / sqrt(D(1.0) - e2 * sinp * sinp);

		chMtxLock(&m_mutex_gps);

		m_gps.lat = gga.lat;
		m_gps.lon = gga.lon;
		m_gps.height = gga.height;
		m_gps.fix_type = gga.fix_type;
		m_gps.sats = gga.n_sat;
		m_gps.ms = gga.t_tow;
		m_gps.x = (v + gga.height) * cosp * cosl;
		m_gps.y = (v + gga.height) * cosp * sinl;
		m_gps.z = (v * (D(1.0) - e2) + gga.height) * sinp;

		// Continue if ENU frame is initialized
		if (m_gps.local_init_done) {
			float dx = (float)(m_gps.x - m_gps.ix);
			float dy = (float)(m_gps.y - m_gps.iy);
			float dz = (float)(m_gps.z - m_gps.iz);

			m_gps.lx = m_gps.r1c1 * dx + m_gps.r1c2 * dy + m_gps.r1c3 * dz;
			m_gps.ly = m_gps.r2c1 * dx + m_gps.r2c2 * dy + m_gps.r2c3 * dz;
			m_gps.lz = m_gps.r3c1 * dx + m_gps.r3c2 * dy + m_gps.r3c3 * dz;

			float px = m_gps.lx;
			float py = m_gps.ly;

			// Apply antenna offset
			const float yaw = pos_get_yaw();
			const float s_yaw = sinf(-yaw* M_PI / 180.0);
			const float c_yaw = cosf(-yaw * M_PI / 180.0);
			px -= c_yaw * main_config.gps_ant_x - s_yaw * main_config.gps_ant_y;
			py -= s_yaw * main_config.gps_ant_x + c_yaw * main_config.gps_ant_y;

			// Correct position
			// Optionally require RTK and good u-blox quality indication.
			if (main_config.gps_comp &&
					(!main_config.gps_req_rtk || (gga.fix_type == 4 || gga.fix_type == 5)) &&
					(!main_config.gps_use_ubx_info || m_ubx_pos_valid)) {

				pos_correction_gnss(px, py, m_gps.lz, m_gps.ms, m_gps.fix_type);

			}
		} else {
			init_gps_local(&m_gps);
			m_gps.local_init_done = true;
		}

		m_gps.update_time = chVTGetSystemTimeX();

		chMtxUnlock(&m_mutex_gps);
	}

	if (gga_res >= 0) // forward NMEA if decoded
		commands_send_nmea(data, strlen(data));
}


static void init_gps_local(GPS_STATE *gps) {
	gps->ix = gps->x;
	gps->iy = gps->y;
	gps->iz = gps->z;

	float so = sinf((float)gps->lon * M_PI / 180.0);
	float co = cosf((float)gps->lon * M_PI / 180.0);
	float sa = sinf((float)gps->lat * M_PI / 180.0);
	float ca = cosf((float)gps->lat * M_PI / 180.0);

	// ENU
	gps->r1c1 = -so;
	gps->r1c2 = co;
	gps->r1c3 = 0.0;

	gps->r2c1 = -sa * co;
	gps->r2c2 = -sa * so;
	gps->r2c3 = ca;

	gps->r3c1 = ca * co;
	gps->r3c2 = ca * so;
	gps->r3c3 = sa;

	gps->lx = 0.0;
	gps->ly = 0.0;
	gps->lz = 0.0;
}
