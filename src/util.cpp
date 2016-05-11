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

#include <unistd.h>
#include <dirent.h>
#include <string.h>
#include <fstream>
#include <util.h>
#include <sensor_log.h>
#include <sensor_config.h>

using std::ifstream;
using std::ofstream;
using std::fstream;
using std::string;

#define PREFIX_EVENT "event"

static bool get_event_num(const string &input_path, string &event_num)
{
	const string event_prefix = PREFIX_EVENT;
	DIR *dir = NULL;
	struct dirent dir_entry;
	struct dirent *result = NULL;
	std::string node_name;
	int error;
	bool find = false;

	dir = opendir(input_path.c_str());
	if (!dir) {
		ERR("Failed to open dir: %s", input_path.c_str());
		return false;
	}

	int prefix_size = event_prefix.size();

	while (true) {
		error = readdir_r(dir, &dir_entry, &result);

		if (error != 0)
			continue;

		if (result == NULL)
			break;

		node_name = std::string(dir_entry.d_name);

		if (node_name.compare(0, prefix_size, event_prefix) != 0)
			continue;

		event_num = node_name.substr(prefix_size, node_name.size() - prefix_size);
		find = true;
		break;
	}

	closedir(dir);

	return find;
}

static bool get_iio_node_info(const string& enable_node_name, const string& device_num, node_info &info)
{
	const string base_dir = string("/sys/bus/iio/devices/iio:device") + device_num + string("/");

	info.data_node_path = string("/dev/iio:device") + device_num;
	info.enable_node_path = base_dir + enable_node_name;
	info.interval_node_path = base_dir + string("sampling_frequency");
	info.buffer_enable_node_path = base_dir + string("buffer/enable");
	info.buffer_length_node_path = base_dir + string("buffer/length");
	info.trigger_node_path = base_dir + string("trigger/current_trigger");

	return true;
}

static bool get_sensorhub_iio_node_info(const string &interval_node_name, const string& device_num, node_info &info)
{
	const string base_dir = string("/sys/bus/iio/devices/iio:device") + device_num + string("/");
	const string hub_dir = "/sys/class/sensors/ssp_sensor/";

	info.data_node_path = string("/dev/iio:device") + device_num;
	info.enable_node_path = hub_dir + string("enable");
	info.interval_node_path = hub_dir + interval_node_name;
	info.buffer_enable_node_path = base_dir + string("buffer/enable");
	info.buffer_length_node_path = base_dir + string("buffer/length");
	return true;
}

static bool get_input_event_node_info(const string& device_num, node_info &info)
{
	string base_dir;
	string event_num;

	base_dir = string("/sys/class/input/input") + device_num + string("/");

	if (!get_event_num(base_dir, event_num))
		return false;

	info.data_node_path = string("/dev/input/event") + event_num;

	info.enable_node_path = base_dir + string("enable");
	info.interval_node_path = base_dir + string("poll_delay");
	return true;
}

static bool get_sensorhub_input_event_node_info(const string &interval_node_name, const string& device_num, node_info &info)
{
	const string base_dir = "/sys/class/sensors/ssp_sensor/";
	string event_num;

	string input_dir = string("/sys/class/input/input") + device_num + string("/");

	if (!get_event_num(input_dir, event_num))
		return false;

	info.data_node_path = string("/dev/input/event") + event_num;
	info.enable_node_path = base_dir + string("enable");
	info.interval_node_path = base_dir + interval_node_name;
	return true;
}

static bool get_node_value(const string &node_path, int &value)
{
	ifstream node(node_path, ifstream::binary);

	if (!node)
		return false;

	node >> value;

	return true;
}

static bool get_input_method(const string &key, int &method, string &device_num)
{
	input_method_info input_info[2] = {
		{INPUT_EVENT_METHOD, "/sys/class/input/", "input"},
		{IIO_METHOD, "/sys/bus/iio/devices/", "iio:device"}
	};

	const int input_info_len = sizeof(input_info)/sizeof(input_info[0]);
	size_t prefix_size;
	std::string name_node, name;
	std::string d_name;
	DIR *dir = NULL;
	struct dirent dir_entry;
	struct dirent *result = NULL;
	int error;
	bool find = false;

	for (int i = 0; i < input_info_len; ++i) {

		prefix_size = input_info[i].prefix.size();

		dir = opendir(input_info[i].dir_path.c_str());
		if (!dir) {
			ERR("Failed to open dir: %s", input_info[i].dir_path.c_str());
			return false;
		}

		find = false;

		while (true) {
			error = readdir_r(dir, &dir_entry, &result);

			if (error != 0)
				continue;

			if (result == NULL)
				break;

			d_name = std::string(dir_entry.d_name);

			if (d_name.compare(0, prefix_size, input_info[i].prefix) != 0)
				continue;

			name_node = input_info[i].dir_path + d_name + string("/name");

			ifstream infile(name_node.c_str());
			if (!infile)
				continue;

			infile >> name;

			if (name != key)
				continue;

			device_num = d_name.substr(prefix_size, d_name.size() - prefix_size);
			find = true;
			method = input_info[i].method;
			break;
		}

		closedir(dir);

		if (find)
			break;
	}

	return find;
}

bool util::set_monotonic_clock(int fd)
{
#ifdef EVIOCSCLOCKID
	int clockId = CLOCK_MONOTONIC;
	if (ioctl(fd, EVIOCSCLOCKID, &clockId) != 0) {
		_E("Fail to set monotonic timestamp for fd[%d]", fd);
		return false;
	}
#endif
	return true;
}

bool util::find_model_id(const string &sensor_type, string &model_id)
{
	std::string dir_path = "/sys/class/sensors/";
	std::string name_node, name;
	std::string d_name;
	DIR *dir = NULL;
	struct dirent dir_entry;
	struct dirent *result = NULL;
	int error;
	bool find = false;

	dir = opendir(dir_path.c_str());
	if (!dir) {
		_D("Failed to open dir: %s", dir_path.c_str());
		return false;
	}

	while (true) {
		error = readdir_r(dir, &dir_entry, &result);

		if (error != 0)
			continue;

		if (result == NULL)
			break;

		d_name = std::string(dir_entry.d_name);

		if ((d_name == ".") || (d_name == "..") || (dir_entry.d_ino == 0))
			continue;

		name_node = dir_path + d_name + string("/name");

		ifstream infile(name_node.c_str());

		if (!infile)
			continue;

		infile >> name;

		if (!config::sensor_config::get_instance().is_supported(sensor_type, name))
			continue;

		model_id = name;
		find = true;
		break;
	}

	closedir(dir);

	return find;
}

bool util::set_enable_node(const string &node_path, bool sensorhub_controlled, bool enable, int enable_bit)
{
	int prev_status, status;

	if (!get_node_value(node_path, prev_status)) {
		ERR("Failed to get node: %s", node_path.c_str());
		return false;
	}

	int _enable_bit = sensorhub_controlled ? enable_bit : 0;

	if (enable)
		status = prev_status | (1 << _enable_bit);
	else
		status = prev_status & (~(1 << _enable_bit));

	if (!set_node_value(node_path, status)) {
		ERR("Failed to set node: %s", node_path.c_str());
		return false;
	}

	return true;
}

unsigned long long util::get_timestamp(void)
{
	struct timespec t;
	clock_gettime(CLOCK_MONOTONIC, &t);
	return ((unsigned long long)(t.tv_sec)*1000000000LL + t.tv_nsec) / 1000;
}

unsigned long long util::get_timestamp(timeval *t)
{
	if (!t) {
		ERR("t is NULL");
		return 0;
	}

	return ((unsigned long long)(t->tv_sec)*1000000LL +t->tv_usec);
}

bool util::is_sensorhub_controlled(const string &key)
{
	string key_node = string("/sys/class/sensors/ssp_sensor/") + key;

	if (access(key_node.c_str(), F_OK) == 0)
		return true;

	return false;
}

bool util::get_node_info(const node_info_query &query, node_info &info)
{
	int method;
	string device_num;

	if (!get_input_method(query.key, method, device_num)) {
		ERR("Failed to get input method for %s", query.key.c_str());
		return false;
	}

	info.method = method;

	if (method == IIO_METHOD) {
		if (query.sensorhub_controlled)
			return get_sensorhub_iio_node_info(query.sensorhub_interval_node_name, device_num, info);
		else
			return get_iio_node_info(query.iio_enable_node_name, device_num, info);
	} else {
		if (query.sensorhub_controlled)
			return get_sensorhub_input_event_node_info(query.sensorhub_interval_node_name, device_num, info);
		else
			return get_input_event_node_info(device_num, info);
	}
}

void util::show_node_info(node_info &info)
{
	if (info.data_node_path.size())
		INFO("Data node: %s", info.data_node_path.c_str());
	if (info.enable_node_path.size())
		INFO("Enable node: %s", info.enable_node_path.c_str());
	if (info.interval_node_path.size())
		INFO("Interval node: %s", info.interval_node_path.c_str());
	if (info.buffer_enable_node_path.size())
		INFO("Buffer enable node: %s", info.buffer_enable_node_path.c_str());
	if (info.buffer_length_node_path.size())
		INFO("Buffer length node: %s", info.buffer_length_node_path.c_str());
	if (info.trigger_node_path.size())
		INFO("Trigger node: %s", info.trigger_node_path.c_str());
}

bool util::set_node_value(const string &node_path, int value)
{
	ofstream node(node_path, ofstream::binary);

	if (!node)
		return false;

	node << value;

	return true;
}

bool util::set_node_value(const string &node_path, unsigned long long value)
{
	ofstream node(node_path, ofstream::binary);

	if (!node)
		return false;

	node << value;

	return true;
}
