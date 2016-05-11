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
#include <sensor_config.h>
#include "hrm_device.h"

#define SENSOR_NAME "SENSOR_HRM"
#define SENSOR_TYPE_HRM "HRM"

#define INPUT_NAME	"hrm_lib_sensor"
#define HRM_SENSORHUB_POLL_NODE_NAME "hrm_poll_delay"

#define DEFAULT_RAW_DATA_UNIT 1

static sensor_info_t sensor_info = {
	id: 0x1,
	name: SENSOR_NAME,
	type: SENSOR_DEVICE_HRM,
	event_type: (SENSOR_DEVICE_HRM << SENSOR_EVENT_SHIFT) | RAW_DATA_EVENT,
	model_name: UNKNOWN_NAME,
	vendor: UNKNOWN_NAME,
	min_range: 0,
	max_range: 1,
	resolution: 1,
	min_interval: 1,
	max_batch_count: 0,
	wakeup_supported: false
};

hrm_device::hrm_device()
: m_node_handle(-1)
, m_hr(0)
, m_spo2(0)
, m_peek_to_peek(0)
, m_snr(0.0f)
, m_raw_data_unit(DEFAULT_RAW_DATA_UNIT)
, m_polling_interval(1000)
, m_fired_time(0)
, m_interval_supported(false)
, m_sensorhub_controlled(false)
{
	double raw_data_unit = DEFAULT_RAW_DATA_UNIT;

	const std::string sensorhub_interval_node_name = HRM_SENSORHUB_POLL_NODE_NAME;
	config::sensor_config &config = config::sensor_config::get_instance();

	node_info_query query;
	node_info info;

	if (!util::find_model_id(SENSOR_TYPE_HRM, m_model_id)) {
		_E("Failed to find model id");
		throw ENXIO;
	}

	query.sensorhub_controlled = m_sensorhub_controlled = util::is_sensorhub_controlled(sensorhub_interval_node_name);
	query.sensor_type = SENSOR_TYPE_HRM;
	query.key = INPUT_NAME;
	query.iio_enable_node_name = "hrm_lib_enable";
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

	if (access(m_interval_node.c_str(), F_OK) == 0)
		m_interval_supported = true;

	if (!config.get(SENSOR_TYPE_HRM, m_model_id, ELEMENT_VENDOR, m_vendor)) {
		_E("[VENDOR] is empty");
		throw ENXIO;
	}

	_I("m_vendor = %s", m_vendor.c_str());

	if (!config.get(SENSOR_TYPE_HRM, m_model_id, ELEMENT_NAME, m_chip_name)) {
		_E("[NAME] is empty");
		throw ENXIO;
	}

	_I("m_chip_name = %s",m_chip_name.c_str());

	if (!config.get(SENSOR_TYPE_HRM, m_model_id, ELEMENT_RAW_DATA_UNIT, raw_data_unit)) {
		_I("[RAW_DATA_UNIT] is empty");
	}

	m_raw_data_unit = (float)(raw_data_unit);
	_I("m_raw_data_unit = %f", m_raw_data_unit);

	m_node_handle = open(m_data_node.c_str(), O_RDONLY);

	if (m_node_handle < 0) {
		_ERRNO(errno, _E, "Failed to open HRM handle");
		throw ENXIO;
	}

	if (m_method != INPUT_EVENT_METHOD)
		throw ENXIO;

	if (!util::set_monotonic_clock(m_node_handle))
		throw ENXIO;

	update_value = [=]() {
		return this->update_value_input_event();
	};

	_I("hrm_device is created!");
}

hrm_device::~hrm_device()
{
	close(m_node_handle);
	m_node_handle = -1;

	_I("hrm_device is destroyed!");
}

int hrm_device::get_poll_fd(void)
{
	return m_node_handle;
}

int hrm_device::get_sensors(const sensor_info_t **sensors)
{
	sensor_info.model_name = m_chip_name.c_str();
	sensor_info.vendor = m_vendor.c_str();
	*sensors = &sensor_info;

	return 1;
}

bool hrm_device::enable(uint32_t id)
{
	util::set_enable_node(m_enable_node, m_sensorhub_controlled, true, SENSORHUB_HRM_LIB_ENABLE_BIT);
	if (m_interval_supported)
		set_interval(id, m_polling_interval);

	m_fired_time = 0;
	_I("Enable HRM sensor");
	return true;
}

bool hrm_device::disable(uint32_t id)
{
	util::set_enable_node(m_enable_node, m_sensorhub_controlled, false, SENSORHUB_HRM_LIB_ENABLE_BIT);

	_I("Disable HRM sensor");
	return true;
}

bool hrm_device::set_interval(uint32_t id, unsigned long val)
{
	unsigned long long polling_interval_ns;

	if (!m_interval_supported)
		return true;

	polling_interval_ns = ((unsigned long long)(val) * 1000llu * 1000llu);

	if (!util::set_node_value(m_interval_node, polling_interval_ns)) {
		_E("Failed to set polling resource: %s", m_interval_node.c_str());
		return false;
	}

	_I("Interval is changed from %dms to %dms", m_polling_interval, val);
	m_polling_interval = val;
	return true;
}

bool hrm_device::update_value_input_event(void)
{
	const float SNR_SIG_FIGS = 10000.0f;
	const int HR_MAX = 300;
	int hrm_raw[4] = {0,};
	unsigned long long fired_time = 0;
	int read_input_cnt = 0;
	const int INPUT_MAX_BEFORE_SYN = 10;
	bool syn = false;

	struct input_event hrm_input;
	_D("hrm event detection!");

	while ((syn == false) && (read_input_cnt < INPUT_MAX_BEFORE_SYN)) {
		int len = read(m_node_handle, &hrm_input, sizeof(hrm_input));
		if (len != sizeof(hrm_input)) {
			_E("hrm_file read fail, read_len = %d", len);
			return false;
		}

		++read_input_cnt;

		if (hrm_input.type == EV_REL) {
			switch (hrm_input.code) {
			case REL_X:
				hrm_raw[0] = (int)hrm_input.value - 1;
				break;
			case REL_Y:
				hrm_raw[1] = (int)hrm_input.value - 1;
				break;
			case REL_Z:
				hrm_raw[2] = (int)hrm_input.value - 1;
				break;
			default:
				_E("hrm_input event[type = %d, code = %d] is unknown.", hrm_input.type, hrm_input.code);
				return false;
				break;
			}
		} else if (hrm_input.type == EV_SYN) {
			syn = true;
			fired_time = util::get_timestamp(&hrm_input.time);
		} else {
			_E("hrm_input event[type = %d, code = %d] is unknown.", hrm_input.type, hrm_input.code);
			return false;
		}
	}

	if ((hrm_raw[0] * m_raw_data_unit) > HR_MAX) {
		_E("Drop abnormal HR: %d", hrm_raw[0]);
		return false;
	}

	m_hr = hrm_raw[0];
	m_peek_to_peek = hrm_raw[1];
	m_snr = ((float)hrm_raw[2] / SNR_SIG_FIGS);
	m_spo2 = 0;
	m_fired_time = fired_time;

	return true;
}

int hrm_device::read_fd(uint32_t **ids)
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

int hrm_device::get_data(uint32_t id, sensor_data_t **data, int *length)
{
	int remains = 1;
	sensor_data_t *sensor_data;
	sensor_data = (sensor_data_t *)malloc(sizeof(sensor_data_t));
	retvm_if(!sensor_data, -ENOMEM, "Memory allocation failed");

	sensor_data->accuracy = SENSOR_ACCURACY_GOOD;
	sensor_data->timestamp = m_fired_time;
	sensor_data->value_count = 4;
	sensor_data->values[0] = m_hr;
	sensor_data->values[1] = m_spo2;
	sensor_data->values[2] = m_peek_to_peek;
	sensor_data->values[3] = m_snr;

	raw_to_base(sensor_data);

	*data = sensor_data;
	*length = sizeof(sensor_data_t);

	return --remains;
}

void hrm_device::raw_to_base(sensor_data_t *data)
{
	data->values[0] = data->values[0] * m_raw_data_unit;
}
