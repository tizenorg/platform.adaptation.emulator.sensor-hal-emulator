/*
 * Copyright (c) 2016 Samsung Electronics Co., Ltd.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#include <fcntl.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>

#include <linux/input.h>
#include <sys/ioctl.h>
#include <poll.h>
#include <math.h>

#include <util.h>
#include <sensor_common.h>
#include <sensor_log.h>
#include "pressure_device.h"

#define SEA_LEVEL_RESOLUTION 0.01
#define SEA_LEVEL_PRESSURE 101325.0
#define SEA_LEVEL_EPSILON 0.00001

#define MODEL_NAME "maru_sensor_pressure_1"
#define VENDOR "Tizen_SDK"
#define RESOLUTION 1
#define RAW_DATA_UNIT 0.0193
#define MIN_INTERVAL 1
#define MIN_RANGE 260
#define MAX_RANGE 1260
#define TEMPERATURE_RESOLUTION 0.05
#define TEMPERATURE_OFFSET 0
#define MAX_BATCH_COUNT 0

#define SENSOR_NAME "SENSOR_PRESSURE"
#define SENSOR_TYPE_PRESSURE "PRESSURE"

#define INPUT_NAME "pressure_sensor"
#define PRESSURE_SENSORHUB_POLL_NODE_NAME "pressure_poll_delay"

static sensor_info_t sensor_info = {
	id: 0x1,
	name: SENSOR_NAME,
	type: SENSOR_DEVICE_PRESSURE,
	event_type: (SENSOR_DEVICE_PRESSURE << SENSOR_EVENT_SHIFT) | RAW_DATA_EVENT,
	model_name: MODEL_NAME,
	vendor: VENDOR,
	min_range: MIN_RANGE,
	max_range: MAX_RANGE,
	resolution: RAW_DATA_UNIT,
	min_interval: MIN_INTERVAL,
	max_batch_count: MAX_BATCH_COUNT,
	wakeup_supported: false
};

pressure_device::pressure_device()
: m_node_handle(-1)
, m_pressure(0)
, m_temperature(0)
, m_sea_level_pressure(SEA_LEVEL_PRESSURE)
, m_polling_interval(1000)
, m_fired_time(0)
, m_sensorhub_controlled(false)
{
	const std::string sensorhub_interval_node_name = PRESSURE_SENSORHUB_POLL_NODE_NAME;

	node_info_query query;
	node_info info;

	query.sensorhub_controlled = m_sensorhub_controlled = util::is_sensorhub_controlled(sensorhub_interval_node_name);
	query.sensor_type = SENSOR_TYPE_PRESSURE;
	query.key = INPUT_NAME;
	query.iio_enable_node_name = "pressure_enable";
	query.sensorhub_interval_node_name = sensorhub_interval_node_name;

	if (!util::get_node_info(query, info)) {
		_E("Failed to get node info");
		throw ENXIO;
	}

	util::show_node_info(info);

	m_method = info.method;
	m_data_node = info.data_node_path;
	m_enable_node = info.enable_node_path;
	m_interval_node = info.interval_node_path;

	m_node_handle = open(m_data_node.c_str(), O_RDONLY);

	if (m_node_handle < 0) {
		_ERRNO(errno, _E, "pressure handle open fail for pressure sensor");
		throw ENXIO;
	}

	if (m_method == INPUT_EVENT_METHOD) {
		if (!util::set_monotonic_clock(m_node_handle))
			throw ENXIO;

		update_value = [=]() {
			return this->update_value_input_event();
		};
	}

	_I("pressure_device is created!");
}

pressure_device::~pressure_device()
{
	close(m_node_handle);
	m_node_handle = -1;

	_I("pressure_device is destroyed!");
}

int pressure_device::get_poll_fd(void)
{
	return m_node_handle;
}

int pressure_device::get_sensors(const sensor_info_t **sensors)
{
	*sensors = &sensor_info;

	return 1;
}

bool pressure_device::enable(uint32_t id)
{
	util::set_enable_node(m_enable_node, m_sensorhub_controlled, true, SENSORHUB_PRESSURE_ENABLE_BIT);
	set_interval(id, m_polling_interval);

	m_fired_time = 0;
	_I("Enable pressure sensor");
	return true;
}

bool pressure_device::disable(uint32_t id)
{
	util::set_enable_node(m_enable_node, m_sensorhub_controlled, false, SENSORHUB_PRESSURE_ENABLE_BIT);

	_I("Disable pressure sensor");
	return true;
}

bool pressure_device::set_interval(uint32_t id, unsigned long val)
{
	unsigned long long polling_interval_ns;

	polling_interval_ns = ((unsigned long long)(val) * 1000llu * 1000llu);

	if (!util::set_node_value(m_interval_node, polling_interval_ns)) {
		_E("Failed to set polling resource: %s", m_interval_node.c_str());
		return false;
	}

	_I("Interval is changed from %dms to %dms", m_polling_interval, val);
	m_polling_interval = val;
	return true;
}

bool pressure_device::update_value_input_event(void)
{
	int pressure_raw[3] = {0,};
	bool pressure = false;
	bool sea_level = false;
	bool temperature = false;
	int read_input_cnt = 0;
	const int INPUT_MAX_BEFORE_SYN = 10;
	unsigned long long fired_time = 0;
	bool syn = false;

	struct input_event pressure_event;
	_D("pressure event detection!");

	while ((syn == false) && (read_input_cnt < INPUT_MAX_BEFORE_SYN)) {
		int len = read(m_node_handle, &pressure_event, sizeof(pressure_event));
		if (len != sizeof(pressure_event)) {
			_E("pressure_file read fail, read_len = %d\n",len);
			return false;
		}

		++read_input_cnt;

		if (pressure_event.type == EV_REL) {
			switch (pressure_event.code) {
				case REL_HWHEEL:
					pressure_raw[0] = (int)pressure_event.value;
					pressure = true;
					break;
				case REL_DIAL:
					pressure_raw[1] = (int)pressure_event.value;
					sea_level = true;
					break;
				case REL_WHEEL:
					pressure_raw[2] = (int)pressure_event.value;
					temperature = true;
					break;
				default:
					_E("pressure_event event[type = %d, code = %d] is unknown.", pressure_event.type, pressure_event.code);
					return false;
					break;
			}
		} else if (pressure_event.type == EV_SYN) {
			syn = true;
			fired_time = util::get_timestamp(&pressure_event.time);
		} else {
			_E("pressure_event event[type = %d, code = %d] is unknown.", pressure_event.type, pressure_event.code);
			return false;
		}
	}

	if (syn == false) {
		_E("EV_SYN didn't come until %d inputs had come", read_input_cnt);
		return false;
	}

	if (pressure)
		m_pressure = pressure_raw[0];
	if (sea_level)
		m_sea_level_pressure = pressure_raw[1];
	if (temperature)
		m_temperature = pressure_raw[2];

	m_fired_time = fired_time;

	_D("m_pressure = %d, sea_level = %d, temperature = %d, time = %lluus", m_pressure, m_sea_level_pressure, m_temperature, m_fired_time);

	return true;
}

int pressure_device::read_fd(uint32_t **ids)
{
	if (!update_value()) {
		_D("Failed to update value");
		return false;
	}

	event_ids.clear();
	event_ids.push_back(sensor_info.id);

	*ids = &event_ids[0];

	return event_ids.size();
}

int pressure_device::get_data(uint32_t id, sensor_data_t **data, int *length)
{
	int remains = 1;
	sensor_data_t *sensor_data;
	sensor_data = (sensor_data_t *)malloc(sizeof(sensor_data_t));
	retvm_if(!sensor_data, -ENOMEM, "Memory allocation failed");

	sensor_data->accuracy = SENSOR_ACCURACY_GOOD;
	sensor_data->timestamp = m_fired_time;
	sensor_data->value_count = 3;
	sensor_data->values[0] = m_pressure;
	sensor_data->values[1] = (float)m_sea_level_pressure;
	sensor_data->values[2] = m_temperature;

	raw_to_base(sensor_data);

	*data = sensor_data;
	*length = sizeof(sensor_data_t);

	return --remains;
}

void pressure_device::raw_to_base(sensor_data_t *data)
{
	data->values[0] = data->values[0] * RAW_DATA_UNIT;
	m_sea_level_pressure = data->values[1] * SEA_LEVEL_RESOLUTION;
	data->values[1] = pressure_to_altitude(data->values[0]);
	data->values[2] = data->values[2] * TEMPERATURE_RESOLUTION + TEMPERATURE_OFFSET;
}

float pressure_device::pressure_to_altitude(float pressure)
{
	float sea_level_pressure = m_sea_level_pressure;

	if (sea_level_pressure < SEA_LEVEL_EPSILON && sea_level_pressure > -SEA_LEVEL_EPSILON)
		sea_level_pressure = SEA_LEVEL_PRESSURE * SEA_LEVEL_RESOLUTION;

	return 44330.0f * (1.0f - pow(pressure/sea_level_pressure, 1.0f/5.255f));
}

