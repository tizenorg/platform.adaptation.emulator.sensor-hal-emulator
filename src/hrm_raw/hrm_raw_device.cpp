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

#include <macro.h>
#include <util.h>
#include <sensor_common.h>
#include <sensor_log.h>
#include <sensor_config.h>

#include "hrm_raw_data_reader_standard.h"
#include "hrm_raw_device.h"

#define SENSOR_NAME "SENSOR_HRM_RAW"
#define SENSOR_TYPE_HRM_RAW "HRM_RAW"

#define INPUT_NAME	"hrm_raw_sensor"
#define HRM_SENSORHUB_POLL_NODE_NAME "hrm_poll_delay"

#define INDEX_HRM_RAW 0x1
#define INDEX_HRM_LED_GREEN 0x2

#define ELEMENT_READER			"READER"
#define POLL_1HZ_MS 1000

static sensor_info_t sensor_info[] = {
	{
		id: INDEX_HRM_RAW,
		name: SENSOR_NAME,
		type: SENSOR_DEVICE_HRM_RAW,
		event_type: (SENSOR_DEVICE_HRM_RAW << SENSOR_EVENT_SHIFT) | RAW_DATA_EVENT,
		model_name: UNKNOWN_NAME,
		vendor: UNKNOWN_NAME,
		min_range: 0,
		max_range: 1,
		resolution: 1,
		min_interval: 1,
		max_batch_count: 0,
		wakeup_supported: false
	},
	{
		id: INDEX_HRM_LED_GREEN,
		name: "HRM LED GREEN SENSOR",
		type: SENSOR_DEVICE_HRM_LED_GREEN,
		event_type: (SENSOR_DEVICE_HRM_LED_GREEN << SENSOR_EVENT_SHIFT) | RAW_DATA_EVENT,
		model_name: UNKNOWN_NAME,
		vendor: UNKNOWN_NAME,
		min_range: 0,
		max_range: 1,
		resolution: 1,
		min_interval: 1,
		max_batch_count: 0,
		wakeup_supported: false
	}
};

hrm_raw_device::hrm_raw_device()
: m_node_handle(-1)
, m_polling_interval(POLL_1HZ_MS)
, m_raw_interval(POLL_1HZ_MS)
, m_led_green_interval(POLL_1HZ_MS)
, m_interval_supported(false)
, m_sensorhub_controlled(false)
, m_enable(0)
, m_reader(NULL)
{
	const std::string sensorhub_interval_node_name = HRM_SENSORHUB_POLL_NODE_NAME;
	config::sensor_config &config = config::sensor_config::get_instance();

	node_info_query query;
	node_info info;

	if (!util::find_model_id(SENSOR_TYPE_HRM_RAW, m_model_id)) {
		_E("Failed to find model id");
		throw ENXIO;
	}

	query.sensorhub_controlled = m_sensorhub_controlled = util::is_sensorhub_controlled(sensorhub_interval_node_name);
	query.sensor_type = SENSOR_TYPE_HRM_RAW;
	query.key = INPUT_NAME;
	query.iio_enable_node_name = "hrm_raw_enable";
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

	if (!config.get(SENSOR_TYPE_HRM_RAW, m_model_id, ELEMENT_VENDOR, m_vendor)) {
		_E("[VENDOR] is empty");
		throw ENXIO;
	}

	_I("m_vendor = %s", m_vendor.c_str());

	if (!config.get(SENSOR_TYPE_HRM_RAW, m_model_id, ELEMENT_NAME, m_chip_name)) {
		_E("[NAME] is empty");
		throw ENXIO;
	}

	_I("m_chip_name = %s",m_chip_name.c_str());

	std::string reader;

	if (!config.get(SENSOR_TYPE_HRM_RAW, m_model_id, ELEMENT_READER, reader)) {
		_E("[READER] is empty");
		throw ENXIO;
	}

	_I("reader = %s", reader.c_str());

	m_node_handle = open(m_data_node.c_str(), O_RDONLY);

	if (m_node_handle < 0) {
		_ERRNO(errno, _E, "hrm raw handle open fail for hrm raw sensor");
		throw ENXIO;
	}

	if (m_method != INPUT_EVENT_METHOD)
		throw ENXIO;

	if (!util::set_monotonic_clock(m_node_handle))
		throw ENXIO;

	m_reader = get_reader(reader);

	if (!m_reader) {
		_E("Not supported HRM sensor: %s", m_model_id.c_str());
		throw ENXIO;
	}

	if (!m_reader->open())
		throw ENXIO;

	_I("hrm_raw_device is created!");
}

hrm_raw_device::~hrm_raw_device()
{
	delete m_reader;
	close(m_node_handle);
	m_node_handle = -1;

	_I("hrm_raw_device is destroyed!");
}

int hrm_raw_device::get_poll_fd()
{
	return m_node_handle;
}

int hrm_raw_device::get_sensors(const sensor_info_t **sensors)
{
	int size = ARRAY_SIZE(sensor_info);

	for (int i = 0; i < size; ++i) {
		sensor_info[i].model_name = m_chip_name.c_str();
		sensor_info[i].vendor = m_vendor.c_str();
	}

	*sensors = sensor_info;

	return size;
}

bool hrm_raw_device::enable(uint32_t id)
{
	++m_enable;

	if (m_enable > 1)
		return true;

	util::set_enable_node(m_enable_node, m_sensorhub_controlled, true, SENSORHUB_HRM_RAW_ENABLE_BIT);
	if (m_interval_supported)
		set_interval(id, m_polling_interval);

	m_data.timestamp = 0;

	m_reader->start();
	_I("Enable HRM Raw sensor");
	return true;
}

bool hrm_raw_device::disable(uint32_t id)
{
	--m_enable;

	if (m_enable > 0)
		return true;

	util::set_enable_node(m_enable_node, m_sensorhub_controlled, false, SENSORHUB_HRM_RAW_ENABLE_BIT);

	m_reader->stop();
	m_enable = 0;
	_I("Disable HRM Raw sensor");
	return true;
}

bool hrm_raw_device::set_interval(uint32_t id, unsigned long val)
{
	unsigned long interval = 100;
	unsigned long long polling_interval_ns;

	if (!m_interval_supported)
		return true;

	if (id == INDEX_HRM_LED_GREEN)
		interval = (val > m_raw_interval)?m_raw_interval:val;
	else
		interval = (val > m_led_green_interval)?m_led_green_interval:val;

	polling_interval_ns = ((unsigned long long)(interval) * 1000llu * 1000llu);

	if (!util::set_node_value(m_interval_node, polling_interval_ns)) {
		_E("Failed to set polling resource: %s", m_interval_node.c_str());
		return false;
	}

	_I("Interval is changed from %dms to %dms", m_polling_interval, interval);
	m_polling_interval = interval;

	if (id == INDEX_HRM_LED_GREEN)
		m_led_green_interval = val;
	else
		m_raw_interval = val;

	return true;
}

int hrm_raw_device::read_fd(uint32_t **ids)
{
	if (!m_reader->get_data(m_node_handle, m_data)) {
		_D("Failed to update value");
		return false;
	}

	int size = ARRAY_SIZE(sensor_info);

	event_ids.clear();

	for (int i = 0; i < size; ++i)
		event_ids.push_back(sensor_info[i].id);

	*ids = &event_ids[0];

	return event_ids.size();
}

int hrm_raw_device::get_data(uint32_t id, sensor_data_t **data, int *length)
{
	int remains = 1;
	sensor_data_t *sensor_data;
	sensor_data = (sensor_data_t *)malloc(sizeof(sensor_data_t));
	retvm_if(!sensor_data, -ENOMEM, "Memory allocation failed");

	sensor_data->accuracy = SENSOR_ACCURACY_GOOD;
	sensor_data->timestamp = m_data.timestamp;

	if (id == INDEX_HRM_LED_GREEN) {
		sensor_data->value_count = 1;
		sensor_data->values[0] = m_data.values[5];
	} else {
		sensor_data->value_count = m_data.value_count;
		memcpy(sensor_data->values, m_data.values, m_data.value_count * sizeof(m_data.values[0]));
	}

	*data = sensor_data;
	*length = sizeof(sensor_data_t);

	return --remains;
}

hrm_raw_data_reader* hrm_raw_device::get_reader(const std::string& reader)
{
	return new(std::nothrow) hrm_raw_data_reader_standard();
}
