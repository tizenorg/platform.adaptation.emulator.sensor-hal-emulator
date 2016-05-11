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
#include "geomag_device.h"

#define SENSOR_NAME "SENSOR_GEOMAGNETIC"
#define SENSOR_TYPE_MAGNETIC	"MAGNETIC"

#define INPUT_NAME	"geomagnetic_sensor"
#define GEOMAG_SENSORHUB_POLL_NODE_NAME "mag_poll_delay"

static sensor_info_t sensor_info = {
	id: 0x1,
	name: SENSOR_NAME,
	type: SENSOR_DEVICE_GEOMAGNETIC,
	event_type: (SENSOR_DEVICE_GEOMAGNETIC << SENSOR_EVENT_SHIFT) | RAW_DATA_EVENT,
	model_name: UNKNOWN_NAME,
	vendor: UNKNOWN_NAME,
	min_range: 0,
	max_range: 0,
	resolution: 0,
	min_interval: 0,
	max_batch_count: 0,
	wakeup_supported: false
};

geomag_device::geomag_device()
: m_node_handle(-1)
, m_x(-1)
, m_y(-1)
, m_z(-1)
, m_hdst(0)
, m_polling_interval(1000)
, m_fired_time(0)
, m_sensorhub_controlled(false)
{
	const std::string sensorhub_interval_node_name = GEOMAG_SENSORHUB_POLL_NODE_NAME;
	config::sensor_config &config = config::sensor_config::get_instance();

	node_info_query query;
	node_info info;

	if (!util::find_model_id(SENSOR_TYPE_MAGNETIC, m_model_id)) {
		_E("Failed to find model id");
		throw ENXIO;

	}

	query.sensorhub_controlled = m_sensorhub_controlled = util::is_sensorhub_controlled(sensorhub_interval_node_name);
	query.sensor_type = SENSOR_TYPE_MAGNETIC;
	query.key = INPUT_NAME;
	query.iio_enable_node_name = "geomagnetic_enable";
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

	if (!config.get(SENSOR_TYPE_MAGNETIC, m_model_id, ELEMENT_VENDOR, m_vendor)) {
		_E("[VENDOR] is empty\n");
		throw ENXIO;
	}

	_I("m_vendor = %s", m_vendor.c_str());

	if (!config.get(SENSOR_TYPE_MAGNETIC, m_model_id, ELEMENT_NAME, m_chip_name)) {
		_E("[NAME] is empty\n");
		throw ENXIO;
	}

	_I("m_chip_name = %s\n",m_chip_name.c_str());

	double min_range;

	if (!config.get(SENSOR_TYPE_MAGNETIC, m_model_id, ELEMENT_MIN_RANGE, min_range)) {
		_E("[MIN_RANGE] is empty\n");
		throw ENXIO;
	}

	m_min_range = (float)min_range;
	_I("m_min_range = %f\n",m_min_range);

	double max_range;

	if (!config.get(SENSOR_TYPE_MAGNETIC, m_model_id, ELEMENT_MAX_RANGE, max_range)) {
		_E("[MAX_RANGE] is empty\n");
		throw ENXIO;
	}

	m_max_range = (float)max_range;
	_I("m_max_range = %f\n",m_max_range);

	double raw_data_unit;

	if (!config.get(SENSOR_TYPE_MAGNETIC, m_model_id, ELEMENT_RAW_DATA_UNIT, raw_data_unit)) {
		_E("[RAW_DATA_UNIT] is empty\n");
		throw ENXIO;
	}

	m_raw_data_unit = (float)(raw_data_unit);
	_I("m_raw_data_unit = %f\n", m_raw_data_unit);

	m_node_handle = open(m_data_node.c_str(), O_RDONLY);

	if (m_node_handle < 0) {
		_ERRNO(errno, _E, "geomag handle open fail for geomag device");
		throw ENXIO;
	}

	if (m_method == INPUT_EVENT_METHOD) {
		if (!util::set_monotonic_clock(m_node_handle))
			throw ENXIO;

		update_value = [=]() {
			return this->update_value_input_event();
		};
	} else {
		if (!info.buffer_length_node_path.empty())
			util::set_node_value(info.buffer_length_node_path, 480);

		if (!info.buffer_enable_node_path.empty())
			util::set_node_value(info.buffer_enable_node_path, 1);

		update_value = [=]() {
			return this->update_value_iio();
		};
	}

	_I("geomag_device is created!\n");
}

geomag_device::~geomag_device()
{
	close(m_node_handle);
	m_node_handle = -1;

	_I("geomag_sensor is destroyed!\n");
}

int geomag_device::get_poll_fd(void)
{
	return m_node_handle;
}

int geomag_device::get_sensors(const sensor_info_t **sensors)
{
	sensor_info.model_name = m_chip_name.c_str();
	sensor_info.vendor = m_vendor.c_str();
	sensor_info.min_range = m_min_range;
	sensor_info.max_range = m_max_range;
	sensor_info.resolution = m_raw_data_unit;
	sensor_info.min_interval = 1;
	sensor_info.max_batch_count = 0;
	*sensors = &sensor_info;

	return 1;
}

bool geomag_device::enable(uint32_t id)
{
	util::set_enable_node(m_enable_node, m_sensorhub_controlled, true, SENSORHUB_GEOMAGNETIC_ENABLE_BIT);
	set_interval(id, m_polling_interval);

	m_fired_time = 0;
	INFO("Enable geomagnetic sensor");
	return true;
}

bool geomag_device::disable(uint32_t id)
{
	util::set_enable_node(m_enable_node, m_sensorhub_controlled, false, SENSORHUB_GEOMAGNETIC_ENABLE_BIT);

	INFO("Disable geomagnetic sensor");
	return true;
}

bool geomag_device::set_interval(uint32_t id, unsigned long val)
{
	unsigned long long polling_interval_ns;

	polling_interval_ns = ((unsigned long long)(val) * 1000llu * 1000llu);

	if (!util::set_node_value(m_interval_node, polling_interval_ns)) {
		ERR("Failed to set polling resource: %s\n", m_interval_node.c_str());
		return false;
	}

	INFO("Interval is changed from %dms to %dms", m_polling_interval, val);
	m_polling_interval = val;
	return true;
}

bool geomag_device::update_value_input_event(void)
{
	int geo_raw[4] = {0,};
	bool x, y, z, hdst;
	int read_input_cnt = 0;
	const int INPUT_MAX_BEFORE_SYN = 10;
	unsigned long long fired_time = 0;
	bool syn = false;

	x = y = z = hdst = false;

	struct input_event geo_input;
	_D("geo event detection!");

	while ((syn == false) && (read_input_cnt < INPUT_MAX_BEFORE_SYN)) {
		int len = read(m_node_handle, &geo_input, sizeof(geo_input));
		if (len != sizeof(geo_input)) {
			_E("geo_file read fail, read_len = %d\n",len);
			return false;
		}

		++read_input_cnt;

		if (geo_input.type == EV_REL) {
			switch (geo_input.code) {
				case REL_RX:
					geo_raw[0] = (int)geo_input.value;
					x = true;
					break;
				case REL_RY:
					geo_raw[1] = (int)geo_input.value;
					y = true;
					break;
				case REL_RZ:
					geo_raw[2] = (int)geo_input.value;
					z = true;
					break;
				case REL_HWHEEL:
					geo_raw[3] = (int)geo_input.value;
					hdst = true;
					break;
				default:
					_E("geo_input event[type = %d, code = %d] is unknown.", geo_input.type, geo_input.code);
					return false;
					break;
			}
		} else if (geo_input.type == EV_SYN) {
			syn = true;
			fired_time = util::get_timestamp(&geo_input.time);
		} else {
			_E("geo_input event[type = %d, code = %d] is unknown.", geo_input.type, geo_input.code);
			return false;
		}
	}

	if (syn == false) {
		_E("EV_SYN didn't come until %d inputs had come", read_input_cnt);
		return false;
	}

	if (x)
		m_x =  geo_raw[0];
	if (y)
		m_y =  geo_raw[1];
	if (z)
		m_z =  geo_raw[2];
	if (hdst)
		m_hdst =  geo_raw[3] - 1; /* accuracy bias: -1 */

	m_fired_time = fired_time;

	_D("m_x = %d, m_y = %d, m_z = %d, m_hdst = %d, time = %lluus", m_x, m_y, m_z, m_hdst, m_fired_time);

	return true;
}


bool geomag_device::update_value_iio(void)
{
	struct {
		int16_t x;
		int16_t y;
		int16_t z;
		int16_t hdst;
		int64_t timestamp;
	} __attribute__((packed)) data;

	struct pollfd pfd;

	pfd.fd = m_node_handle;
	pfd.events = POLLIN | POLLERR;
	pfd.revents = 0;

	int ret = poll(&pfd, 1, -1);

	if (ret == -1) {
		_ERRNO(errno, _E, "Failed to poll from m_node_handle:%d", m_node_handle);
		return false;
	} else if (!ret) {
		_E("poll timeout m_node_handle:%d", m_node_handle);
		return false;
	}

	if (pfd.revents & POLLERR) {
		_E("poll exception occurred! m_node_handle:%d", m_node_handle);
		return false;
	}

	if (!(pfd.revents & POLLIN)) {
		_E("poll nothing to read! m_node_handle:%d, pfd.revents = %d", m_node_handle, pfd.revents);
		return false;
	}

	int len = read(m_node_handle, &data, sizeof(data));

	if (len != sizeof(data)) {
		_E("Failed to read data, m_node_handle:%d read_len:%d", m_node_handle, len);
		return false;
	}

	m_x = data.x;
	m_y = data.y;
	m_z = data.z;
	m_hdst = data.hdst - 1;
	m_fired_time = data.timestamp;

	_D("m_x = %d, m_y = %d, m_z = %d, m_hdst = %d, time = %lluus", m_x, m_y, m_z, m_hdst, m_fired_time);

	return true;
}

int geomag_device::read_fd(uint32_t **ids)
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

int geomag_device::get_data(uint32_t id, sensor_data_t **data, int *length)
{
	int remains = 1;
	sensor_data_t *sensor_data;
	sensor_data = (sensor_data_t *)malloc(sizeof(sensor_data_t));
	retvm_if(!sensor_data, -ENOMEM, "Memory allocation failed");

	sensor_data->accuracy = (m_hdst == 1)? 0 : m_hdst;
	sensor_data->timestamp = m_fired_time;
	sensor_data->value_count = 4;
	sensor_data->values[0] = m_x;
	sensor_data->values[1] = m_y;
	sensor_data->values[2] = m_z;
	sensor_data->values[3] = (m_hdst == 1)? 0 : m_hdst;

	raw_to_base(sensor_data);

	*data = sensor_data;
	*length = sizeof(sensor_data_t);

	return --remains;
}

void geomag_device::raw_to_base(sensor_data_t *data)
{
	data->values[0] = data->values[0] * m_raw_data_unit;
	data->values[1] = data->values[1] * m_raw_data_unit;
	data->values[2] = data->values[2] * m_raw_data_unit;
}
