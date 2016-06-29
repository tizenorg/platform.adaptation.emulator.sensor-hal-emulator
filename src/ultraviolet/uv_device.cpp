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

#include "uv_device.h"

#define MODEL_NAME "maru_sensor_uv_1"
#define VENDOR "Tizen_SDK"
#define MIN_RANGE 0
#define MAX_RANGE 15
#define RESOLUTION 1
#define RAW_DATA_UNIT 0.1
#define MIN_INTERVAL 1
#define MAX_BATCH_COUNT 0

#define SENSOR_NAME "ULTRAVIOLET_SENSOR"
#define SENSOR_TYPE_ULTRAVIOLET		"ULTRAVIOLET"

#define INPUT_NAME "uv_sensor"
#define UV_SENSORHUB_POLL_NODE_NAME "uv_poll_dealy"

#define IIO_ENABLE_NAME "uv_enable"

#define BIAS	1

static sensor_info_t sensor_info = {
	id: 0x1,
	name: SENSOR_NAME,
	type: SENSOR_DEVICE_ULTRAVIOLET,
	event_type: (SENSOR_DEVICE_ULTRAVIOLET << SENSOR_EVENT_SHIFT) | RAW_DATA_EVENT,
	model_name: MODEL_NAME,
	vendor: VENDOR,
	min_range: MIN_RANGE,
	max_range: MAX_RANGE,
	resolution: RAW_DATA_UNIT,
	min_interval: MIN_INTERVAL,
	max_batch_count: MAX_BATCH_COUNT,
	wakeup_supported: false
};

uv_device::uv_device()
: m_node_handle(-1)
, m_ultraviolet(0)
, m_polling_interval(1000)
, m_fired_time(0)
, m_sensorhub_controlled(false)
{
	const std::string sensorhub_interval_node_name = UV_SENSORHUB_POLL_NODE_NAME;

	node_info_query query;
	node_info info;

	query.sensorhub_controlled = m_sensorhub_controlled = util::is_sensorhub_controlled(sensorhub_interval_node_name);
	query.sensor_type = SENSOR_TYPE_ULTRAVIOLET;
	query.key = INPUT_NAME;
	query.iio_enable_node_name = IIO_ENABLE_NAME;
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
		_ERRNO(errno, _E, "uv handle open fail for uv device");
		throw ENXIO;
	}

	if (m_method != INPUT_EVENT_METHOD)
		throw ENXIO;

	if (!util::set_monotonic_clock(m_node_handle))
		throw ENXIO;

	update_value = [=]() {
		return this->update_value_input_event();
	};

	_I("uv_device is created!");
}

uv_device::~uv_device()
{
	close(m_node_handle);
	m_node_handle = -1;

	_I("uv_sensor is destroyed!");
}

int uv_device::get_poll_fd(void)
{
	return m_node_handle;
}

int uv_device::get_sensors(const sensor_info_t **sensors)
{
	*sensors = &sensor_info;

	return 1;
}

bool uv_device::enable(uint32_t id)
{
	util::set_enable_node(m_enable_node, m_sensorhub_controlled, true, SENSORHUB_UV_SENSOR);
	set_interval(id, m_polling_interval);

	m_fired_time = 0;
	_I("Enable uverometer sensor");
	return true;
}

bool uv_device::disable(uint32_t id)
{
	util::set_enable_node(m_enable_node, m_sensorhub_controlled, false, SENSORHUB_UV_SENSOR);

	_I("Disable uverometer sensor");
	return true;
}

bool uv_device::set_interval(uint32_t id, unsigned long val)
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

bool uv_device::update_value_input_event(void)
{
	int ultraviolet_raw = -1;
	bool ultraviolet = false;
	int read_input_cnt = 0;
	const int INPUT_MAX_BEFORE_SYN = 10;
	unsigned long long fired_time = 0;
	bool syn = false;

	struct input_event ultraviolet_event;
	_D("ultraviolet event detection!");

	while ((syn == false) && (read_input_cnt < INPUT_MAX_BEFORE_SYN)) {
		int len = read(m_node_handle, &ultraviolet_event, sizeof(ultraviolet_event));
		if (len != sizeof(ultraviolet_event)) {
			_E("ultraviolet file read fail, read_len = %d\n",len);
			return false;
		}

		++read_input_cnt;

		if (ultraviolet_event.type == EV_REL && ultraviolet_event.code == REL_MISC) {
			ultraviolet_raw = (int)ultraviolet_event.value - BIAS;
			ultraviolet = true;
		} else if (ultraviolet_event.type == EV_SYN) {
			syn = true;
			fired_time = util::get_timestamp(&ultraviolet_event.time);
		} else {
			_E("ultraviolet event[type = %d, code = %d] is unknown.", ultraviolet_event.type, ultraviolet_event.code);
			return false;
		}
	}

	if (ultraviolet)
		m_ultraviolet = ultraviolet_raw;

	m_fired_time = fired_time;

	_D("m_ultraviolet = %d, time = %lluus", m_ultraviolet, m_fired_time);

	return true;
}

int uv_device::read_fd(uint32_t **ids)
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

int uv_device::get_data(uint32_t id, sensor_data_t **data, int *length)
{
	sensor_data_t *sensor_data;
	sensor_data = (sensor_data_t *)malloc(sizeof(sensor_data_t));
	retvm_if(!sensor_data, -ENOMEM, "Memory allocation failed");

	sensor_data->accuracy = SENSOR_ACCURACY_GOOD;
	sensor_data->timestamp = m_fired_time;
	sensor_data->value_count = 1;
	sensor_data->values[0] = m_ultraviolet;

	raw_to_base(sensor_data);

	*data = sensor_data;
	*length = sizeof(sensor_data_t);

	return 1;
}

void uv_device::raw_to_base(sensor_data_t *data)
{
	data->values[0] = data->values[0] * RAW_DATA_UNIT;
}
