#pragma once

#include <cstdint>

// FIXME: Type is missing, copied over from PX4.
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


