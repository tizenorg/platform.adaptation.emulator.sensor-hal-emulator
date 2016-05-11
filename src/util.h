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

#ifndef _SENSOR_UTIL_H_
#define _SENSOR_UTIL_H_

#include <sys/time.h>
#include <string>

typedef struct {
	int method;
	std::string data_node_path;
	std::string enable_node_path;
	std::string interval_node_path;
	std::string buffer_enable_node_path;
	std::string buffer_length_node_path;
	std::string trigger_node_path;
} node_info;

typedef struct {
	bool sensorhub_controlled;
	std::string sensor_type;
	std::string key;
	std::string iio_enable_node_name;
	std::string sensorhub_interval_node_name;
} node_info_query;

enum input_method {
	IIO_METHOD = 0,
	INPUT_EVENT_METHOD = 1,
};

typedef struct {
	int method;
	std::string dir_path;
	std::string prefix;
} input_method_info;

namespace util {
	bool set_monotonic_clock(int fd);

	bool find_model_id(const std::string &sensor_type, std::string &model_id);
	bool set_enable_node(const std::string &node_path, bool sensorhub_controlled, bool enable, int enable_bit = 0);

	unsigned long long get_timestamp(void);
	unsigned long long get_timestamp(timeval *t);

	bool is_sensorhub_controlled(const std::string &key);
	bool get_node_info(const node_info_query &query, node_info &info);
	void show_node_info(node_info &info);
	bool set_node_value(const std::string &node_path, int value);
	bool set_node_value(const std::string &node_path, unsigned long long value);
}
#endif /*_SENSOR_UTIL_H_*/
