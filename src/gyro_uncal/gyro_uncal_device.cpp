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

#include <util.h>
#include <sensor_common.h>
#include <sensor_log.h>

#include "gyro_uncal_device.h"

#define MODEL_NAME "maru_sensor_gyro_1"
#define VENDOR "Tizen_SDK"
#define RESOLUTION 16
#define RAW_DATA_UNIT 1000
#define MIN_INTERVAL 1
#define MAX_BATCH_COUNT 0

#define SENSOR_NAME "SENSOR_GYROSCOPE_UNCALIBRATED"
#define SENSOR_TYPE_GYRO_UNCAL  "GYRO"

#define INPUT_NAME	"uncal_gyro_sensor"
#define GYRO_UNCAL_SENSORHUB_POLL_NODE_NAME "uncal_gyro_poll_delay"

#define DPS_TO_MDPS 1000
#define RAW_DATA_TO_DPS_UNIT(X) ((float)(X)/((float)DPS_TO_MDPS))
#define MIN_RANGE(RES) (-((1 << (RES))/2))
#define MAX_RANGE(RES) (((1 << (RES))/2)-1)

static sensor_info_t sensor_info = {
	id: 0x1,
	name: SENSOR_NAME,
	type: SENSOR_DEVICE_GYROSCOPE_UNCAL,
	event_type: (SENSOR_DEVICE_GYROSCOPE_UNCAL << SENSOR_EVENT_SHIFT) | RAW_DATA_EVENT,
	model_name: MODEL_NAME,
	vendor: VENDOR,
	min_range: MIN_RANGE(RESOLUTION) * RAW_DATA_TO_DPS_UNIT(RAW_DATA_UNIT),
	max_range: MAX_RANGE(RESOLUTION) * RAW_DATA_TO_DPS_UNIT(RAW_DATA_UNIT),
	resolution: RAW_DATA_TO_DPS_UNIT(RAW_DATA_UNIT),
	min_interval: MIN_INTERVAL,
	max_batch_count: MAX_BATCH_COUNT,
	wakeup_supported: false
};

gyro_uncal_device::gyro_uncal_device()
: m_node_handle(-1)
, m_x(-1)
, m_y(-1)
, m_z(-1)
, m_x_offset(-1)
, m_y_offset(-1)
, m_z_offset(-1)
, m_polling_interval(1000)
, m_fired_time(0)
, m_sensorhub_controlled(false)
{
	const std::string sensorhub_interval_node_name = GYRO_UNCAL_SENSORHUB_POLL_NODE_NAME;

	node_info_query query;
	node_info info;

	query.sensorhub_controlled = m_sensorhub_controlled = util::is_sensorhub_controlled(sensorhub_interval_node_name);
	query.sensor_type = SENSOR_TYPE_GYRO_UNCAL;
	query.key = INPUT_NAME;
	query.iio_enable_node_name = "uncal_gyro_enable";
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
		_ERRNO(errno, _E, "gyro_uncal handle open fail for gyro_uncal device");
		throw ENXIO;
	}

	if (m_method == INPUT_EVENT_METHOD) {
		if (!util::set_monotonic_clock(m_node_handle))
			throw ENXIO;

		update_value = [=]() {
			return this->update_value_input_event();
		};
	}

	_I("gyro_uncal_device is created!");
}

gyro_uncal_device::~gyro_uncal_device()
{
	close(m_node_handle);
	m_node_handle = -1;

	_I("gyro_uncal_device is destroyed!");
}

int gyro_uncal_device::get_poll_fd(void)
{
	return m_node_handle;
}

int gyro_uncal_device::get_sensors(const sensor_info_t **sensors)
{
	*sensors = &sensor_info;

	return 1;
}

bool gyro_uncal_device::enable(uint32_t id)
{
	util::set_enable_node(m_enable_node, m_sensorhub_controlled, true, SENSORHUB_GYRO_UNCALIB_ENABLE_BIT);
	set_interval(id, m_polling_interval);

	m_fired_time = 0;
	_I("Enable gyroscope uncalibration sensor");
	return true;
}

bool gyro_uncal_device::disable(uint32_t id)
{
	util::set_enable_node(m_enable_node, m_sensorhub_controlled, false, SENSORHUB_GYRO_UNCALIB_ENABLE_BIT);

	_I("Disable gyroscope uncalibration sensor");
	return true;
}

bool gyro_uncal_device::set_interval(uint32_t id, unsigned long val)
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

bool gyro_uncal_device::update_value_input_event(void)
{
	int gyro_uncal_raw[6] = {0,};
	bool x,y,z,x_offset,y_offset,z_offset;
	int read_input_cnt = 0;
	const int INPUT_MAX_BEFORE_SYN = 10;
	unsigned long long fired_time = 0;
	bool syn = false;

	x = y = z = x_offset = y_offset = z_offset = false;

	struct input_event gyro_uncal_input;
	_D("uncal gyro event detection!");

	while ((syn == false) && (read_input_cnt < INPUT_MAX_BEFORE_SYN)) {
		int len = read(m_node_handle, &gyro_uncal_input, sizeof(gyro_uncal_input));
		if (len != sizeof(gyro_uncal_input)) {
			_E("gyro_file read fail, read_len = %d",len);
			return false;
		}

		++read_input_cnt;

		if (gyro_uncal_input.type == EV_REL) {
			switch (gyro_uncal_input.code) {
			case REL_RX:
				gyro_uncal_raw[0] = (int)gyro_uncal_input.value;
				x = true;
				break;
			case REL_RY:
				gyro_uncal_raw[1] = (int)gyro_uncal_input.value;
				y = true;
				break;
			case REL_RZ:
				gyro_uncal_raw[2] = (int)gyro_uncal_input.value;
				z = true;
				break;
			case REL_HWHEEL:
				gyro_uncal_raw[3] = (int)gyro_uncal_input.value;
				x_offset = true;
				break;
			case REL_DIAL:
				gyro_uncal_raw[4] = (int)gyro_uncal_input.value;
				y_offset = true;
				break;
			case REL_WHEEL:
				gyro_uncal_raw[5] = (int)gyro_uncal_input.value;
				z_offset= true;
				break;
			default:
				_E("gyro_uncal_input event[type = %d, code = %d] is unknown.", gyro_uncal_input.type, gyro_uncal_input.code);
				return false;
				break;
			}
		} else if (gyro_uncal_input.type == EV_SYN) {
			syn = true;
			fired_time = util::get_timestamp(&gyro_uncal_input.time);
		} else {
			_E("gyro_uncal_input event[type = %d, code = %d] is unknown.", gyro_uncal_input.type, gyro_uncal_input.code);
			return false;
		}
	}

	if (syn == false) {
		_E("EV_SYN didn't come until %d inputs had come", read_input_cnt);
		return false;
	}

	if (x)
		m_x =  gyro_uncal_raw[0];
	if (y)
		m_y =  gyro_uncal_raw[1];
	if (z)
		m_z =  gyro_uncal_raw[2];
	if (x_offset)
		m_x_offset=  gyro_uncal_raw[3];
	if (y_offset)
		m_y_offset=  gyro_uncal_raw[4];
	if (z_offset)
		m_z_offset=  gyro_uncal_raw[5];

	m_fired_time = fired_time;

	_D("m_x = %d, m_y = %d, m_z = %d, m_x_offset = %d, m_y_offset = %d, m_z_offset = %d, time = %lluus", m_x, m_y, m_z, m_x_offset, m_y_offset, m_z_offset, m_fired_time);

	return true;
}

int gyro_uncal_device::read_fd(uint32_t **ids)
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

int gyro_uncal_device::get_data(uint32_t id, sensor_data_t **data, int *length)
{
	int remains = 1;
	sensor_data_t *sensor_data;
	sensor_data = (sensor_data_t *)malloc(sizeof(sensor_data_t));
	retvm_if(!sensor_data, -ENOMEM, "Memory allocation failed");

	sensor_data->accuracy = SENSOR_ACCURACY_GOOD;
	sensor_data->timestamp = m_fired_time;
	sensor_data->value_count = 6;
	sensor_data->values[0] = m_x;
	sensor_data->values[1] = m_y;
	sensor_data->values[2] = m_z;
	sensor_data->values[2] = m_x_offset;
	sensor_data->values[2] = m_y_offset;
	sensor_data->values[2] = m_z_offset;

	raw_to_base(sensor_data);

	*data = sensor_data;
	*length = sizeof(sensor_data_t);

	return --remains;
}

void gyro_uncal_device::raw_to_base(sensor_data_t *data)
{
	data->values[0] = data->values[0] * RAW_DATA_TO_DPS_UNIT(RAW_DATA_UNIT);
	data->values[1] = data->values[1] * RAW_DATA_TO_DPS_UNIT(RAW_DATA_UNIT);
	data->values[2] = data->values[2] * RAW_DATA_TO_DPS_UNIT(RAW_DATA_UNIT);
	data->values[3] = data->values[3] * RAW_DATA_TO_DPS_UNIT(RAW_DATA_UNIT);
	data->values[4] = data->values[4] * RAW_DATA_TO_DPS_UNIT(RAW_DATA_UNIT);
	data->values[5] = data->values[5] * RAW_DATA_TO_DPS_UNIT(RAW_DATA_UNIT);
}
