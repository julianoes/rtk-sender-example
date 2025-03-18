#pragma once

#include <cstdint>
#include <cstdio>
#include <ctime>

#define GPS_INFO(...) printf(__VA_ARGS__)
#define GPS_WARN(...) printf(__VA_ARGS__)
#define GPS_ERR(...) printf(__VA_ARGS__)

#define M_DEG_TO_RAD 		(M_PI / 180.0)
#define M_RAD_TO_DEG 		(180.0 / M_PI)
#define M_DEG_TO_RAD_F 		0.01745329251994f
#define M_RAD_TO_DEG_F 		57.2957795130823f


#include <sys/time.h>

using gps_abstime = uint64_t;

static inline gps_abstime gps_absolute_time() {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return tv.tv_sec * 1'000'000 + tv.tv_usec;
}

// FIXME: Types are missing, copied over from PX4.

struct sensor_gps_s {
	uint64_t timestamp;
	uint64_t time_utc_usec;
	uint32_t device_id;
	int32_t latitude_deg;
	int32_t longitude_deg;
	int32_t altitude_msl_m;
	int32_t altitude_ellipsoid_m;
	float s_variance_m_s;
	float c_variance_rad;
	float eph;
	float epv;
	float hdop;
	float vdop;
	int32_t noise_per_ms;
	int32_t jamming_indicator;
	float vel_m_s;
	float vel_n_m_s;
	float vel_e_m_s;
	float vel_d_m_s;
	float cog_rad;
	int32_t timestamp_time_relative;
	float heading;
	float heading_offset;
	float heading_accuracy;
	uint16_t automatic_gain_control;
	uint8_t fix_type;
	uint8_t jamming_state;
	bool vel_ned_valid;
	uint8_t satellites_used;
	uint8_t _padding0[2]; // required for logger
	uint8_t spoofing_state;
	bool rtcm_crc_failed;
	bool rtcm_msg_used;
};

struct satellite_info_s {
	uint64_t timestamp;
	uint8_t count;
	uint8_t svid[20];
	uint8_t used[20];
	uint8_t elevation[20];
	uint8_t azimuth[20];
	uint8_t snr[20];
	uint8_t prn[20];
	uint8_t _padding0[7]; // required for logger

    static constexpr uint8_t SAT_INFO_MAX_SATELLITES = 20;
};

struct sensor_gnss_relative_s {
	uint64_t timestamp;
	uint64_t timestamp_sample;
	uint64_t time_utc_usec;
	uint32_t device_id;
	float position[3];
	float position_accuracy[3];
	float heading;
	float heading_accuracy;
	float position_length;
	float accuracy_length;
	uint16_t reference_station_id;
	bool gnss_fix_ok;
	bool differential_solution;
	bool relative_position_valid;
	bool carrier_solution_floating;
	bool carrier_solution_fixed;
	bool moving_base_mode;
	bool reference_position_miss;
	bool reference_observations_miss;
	bool heading_valid;
	bool relative_position_normalized;
};

