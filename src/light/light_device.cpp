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

#include <math.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>

#include <linux/input.h>
#include <sys/ioctl.h>
#include <poll.h>

#include <macro.h>
#include <util.h>
#include <sensor_common.h>
#include <sensor_log.h>
#include <sensor_config.h>
#include "light_device.h"

#define BIAS	1

#define SENSOR_NAME "SENSOR_LIGHT"

/* ADC value received from Kernel */
#define MODEL_ID_CAPELLA		"CM36686"

#define SENSOR_TYPE_LIGHT		"LIGHT"

#define INPUT_NAME	"light_sensor"
#define LIGHT_SENSORHUB_POLL_NODE_NAME "light_poll_delay"

const static int light_level[] = {0, 1, 165, 288, 497, 869, 1532, 2692, 4692, 8280, 21428, 65535, 137852};

static sensor_info_t sensor_info = {
	id: 0x1,
	name: SENSOR_NAME,
	type: SENSOR_DEVICE_LIGHT,
	event_type: (SENSOR_DEVICE_LIGHT << SENSOR_EVENT_SHIFT) | RAW_DATA_EVENT,
	model_name: UNKNOWN_NAME,
	vendor: UNKNOWN_NAME,
	min_range: 0,
	max_range: 65536,
	resolution: 1,
	min_interval: 1,
	max_batch_count: 0,
	wakeup_supported: false
};

light_device::light_device()
: m_node_handle(-1)
, m_lux(-1)
, m_polling_interval(1000)
, m_fired_time(0)
, m_sensorhub_controlled(false)
{
	const std::string sensorhub_interval_node_name = LIGHT_SENSORHUB_POLL_NODE_NAME;
	config::sensor_config &config = config::sensor_config::get_instance();

	node_info_query query;
	node_info info;

	if (!util::find_model_id(SENSOR_TYPE_LIGHT, m_model_id)) {
		_E("Failed to find model id");
		throw ENXIO;
	}

	query.sensorhub_controlled = m_sensorhub_controlled = util::is_sensorhub_controlled(sensorhub_interval_node_name);
	query.sensor_type = SENSOR_TYPE_LIGHT;
	query.key = INPUT_NAME;
	query.iio_enable_node_name = "light_enable";
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

	if (!config.get(SENSOR_TYPE_LIGHT, m_model_id, ELEMENT_VENDOR, m_vendor)) {
		_E("[VENDOR] is empty");
		throw ENXIO;
	}

	_I("m_vendor = %s", m_vendor.c_str());

	if (!config.get(SENSOR_TYPE_LIGHT, m_model_id, ELEMENT_NAME, m_chip_name)) {
		_E("[NAME] is empty");
		throw ENXIO;
	}

	_I("m_chip_name = %s",m_chip_name.c_str());

	m_node_handle = open(m_data_node.c_str(), O_RDONLY);

	if (m_node_handle < 0) {
		_ERRNO(errno, _E, "light handle open fail for light device");
		throw ENXIO;
	}

	if (m_method == INPUT_EVENT_METHOD) {
		if (!util::set_monotonic_clock(m_node_handle))
			throw ENXIO;

		if (m_chip_name == MODEL_ID_CAPELLA) {
			update_value = [=]() {
				return this->update_value_adc();
			};
		} else {
			update_value = [=]() {
				return this->update_value_lux();
			};
		}
	}

	_I("light_device is created!");
}

light_device::~light_device()
{
	close(m_node_handle);
	m_node_handle = -1;

	_I("light_device is destroyed!");
}

int light_device::get_poll_fd()
{
	return m_node_handle;
}

int light_device::get_sensors(const sensor_info_t **sensors)
{
	sensor_info.model_name = m_chip_name.c_str();
	sensor_info.vendor = m_vendor.c_str();
	*sensors = &sensor_info;

	return 1;
}

bool light_device::enable(uint32_t id)
{
	util::set_enable_node(m_enable_node, m_sensorhub_controlled, true, SENSORHUB_LIGHT_ENABLE_BIT);
	set_interval(id, m_polling_interval);

	m_fired_time = 0;
	_I("Enable light sensor");
	return true;
}

bool light_device::disable(uint32_t id)
{
	util::set_enable_node(m_enable_node, m_sensorhub_controlled, false, SENSORHUB_LIGHT_ENABLE_BIT);

	INFO("Disable light sensor");
	return true;
}

bool light_device::set_interval(uint32_t id, unsigned long val)
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

bool light_device::update_value_adc(void)
{
	int light_raw[2] = {0,};
	int als = -1;
	int w = -1;
	int lux = -1;
	float i_cf = 0.0f;
	bool adc, white;
	int read_input_cnt = 0;
	const int INPUT_MAX_BEFORE_SYN = 10;
	unsigned long long fired_time = 0;
	bool syn = false;

	adc = white = false;

	struct input_event light_input;
	_D("light event detection!");

	while ((syn == false) && (read_input_cnt < INPUT_MAX_BEFORE_SYN)) {
		int len = read(m_node_handle, &light_input, sizeof(light_input));
		if (len != sizeof(light_input)) {
			_E("light_file read fail, read_len = %d",len);
			return false;
		}

		++read_input_cnt;

		if (light_input.type == EV_REL) {
			switch (light_input.code) {
			case REL_DIAL:
				light_raw[0] = (int)light_input.value - BIAS;
				adc = true;
				break;
			case REL_WHEEL:
				light_raw[1] = (int)light_input.value - BIAS;
				white = true;
				break;
			default:
				_E("light_input event[type = %d, code = %d] is unknown.", light_input.type, light_input.code);
				return false;
				break;
			}
		} else if (light_input.type == EV_SYN) {
			syn = true;
			fired_time = util::get_timestamp(&light_input.time);
		} else {
			_E("light_input event[type = %d, code = %d] is unknown.", light_input.type, light_input.code);
			return false;
		}
	}

	if (syn == false) {
		_E("EV_SYN didn't come until %d inputs had come", read_input_cnt);
		return false;
	}

	if (adc && white) {
		als =  light_raw[0];
		w = (light_raw[1])? light_raw[1] : 1;
		i_cf = als / (float) w;
		if (i_cf >= 0.33f) {
			lux = 0.6985f * pow(als, 0.9943f);
		} else {
			lux = 0.25f * pow(als, 1.0552f);
		}
	}

	_D("update_value_lux, lux : %d", lux);

	m_fired_time = fired_time;
	m_lux = lux;

	return true;
}

bool light_device::update_value_lux(void)
{
	int lux = -1;
	struct input_event light_event;
	_D("light event detection!");

	int len = read(m_node_handle, &light_event, sizeof(light_event));
	if (len == -1) {
		_ERRNO(errno, _E, "Failed to read from m_node_handle");
		return false;
	}

	if (light_event.type == EV_ABS && light_event.code == ABS_MISC) {
		lux = light_event.value;
	} else if (light_event.type == EV_REL && light_event.code == REL_RX) {
		lux = light_event.value - BIAS;
	} else {
		_D("light input event[type = %d, code = %d] is unknown.", light_event.type, light_event.code);
		return false;
	}

	_D("read event, len : %d, type : %x, code : %x, value : %x",
		len, light_event.type, light_event.code, light_event.value);

	_D("update_value_lux, lux : %d", lux);

	m_lux = lux;
	m_fired_time = util::get_timestamp(&light_event.time);

	return true;
}

int light_device::read_fd(uint32_t **ids)
{
	if (!update_value()) {
		DBG("Failed to update value");
		return false;
	}

	event_ids.clear();
	event_ids.push_back(sensor_info.id);

	*ids = &event_ids[0];

	return event_ids.size();
}

int light_device::get_data(uint32_t id, sensor_data_t **data, int *length)
{
	int remains = 1;
	sensor_data_t *sensor_data;
	sensor_data = (sensor_data_t *)malloc(sizeof(sensor_data_t));
	retvm_if(!sensor_data, -ENOMEM, "Memory allocation failed");

	sensor_data->accuracy = SENSOR_ACCURACY_GOOD;
	sensor_data->timestamp = m_fired_time;
	sensor_data->value_count = 1;
	sensor_data->values[0] = (float)m_lux;

	*data = sensor_data;
	*length = sizeof(sensor_data_t);

	return --remains;
}

int light_device::adc_to_light_level(int adc)
{
	int level_cnt = ARRAY_SIZE(light_level) - 1;

	for (int i = 0; i < level_cnt; ++i) {
		if (adc >= light_level[i] && adc < light_level[i + 1])
			return i;
	}

	return -1;
}
