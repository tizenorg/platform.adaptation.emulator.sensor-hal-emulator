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

#ifndef _GEOMAG_DEVICE_H_
#define _GEOMAG_DEVICE_H_

#include <sensor/sensor_hal.h>
#include <string>
#include <vector>
#include <functional>

class geomag_device : public sensor_device {
public:
	geomag_device();
	virtual ~geomag_device();

	int get_poll_fd(void);
	int get_sensors(const sensor_info_t **sensors);

	bool enable(uint32_t id);
	bool disable(uint32_t id);

	bool set_interval(uint32_t id, unsigned long val);

	int read_fd(uint32_t **ids);
	int get_data(uint32_t id, sensor_data_t **data, int *length);

private:
	int m_node_handle;
	int m_x;
	int m_y;
	int m_z;
	int m_hdst;
	unsigned long m_polling_interval;
	unsigned long long m_fired_time;
	bool m_sensorhub_controlled;

	int m_method;
	std::string m_data_node;
	std::string m_enable_node;
	std::string m_interval_node;

	long a_x;
	long a_y;
	long a_z;

	std::function<bool (void)> update_value;

	std::vector<uint32_t> event_ids;

	bool update_value_input_event(void);
	bool update_value_iio(void);

	void raw_to_base(sensor_data_t *data);
};
#endif /*_GEOMAG_DEVICE_H_*/
