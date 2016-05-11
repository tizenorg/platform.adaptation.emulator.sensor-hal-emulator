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
#include <linux/input.h>
#include <util.h>
#include <sensor_log.h>
#include <sensor_common.h>
#include "hrm_raw_data_reader_standard.h"

#define INPUT_MAX_BEFORE_SYN	20
#define INPUT_EVENT_BIAS	1
#define INPUT_VALUE_COUNT	10

hrm_raw_data_reader_standard::hrm_raw_data_reader_standard()
{
}

hrm_raw_data_reader_standard::~hrm_raw_data_reader_standard()
{
}

bool hrm_raw_data_reader_standard::get_data(int handle, sensor_data_t &data)
{
	bool syn = false;
	int read_input_cnt = 0;
	int index = 0;
	struct input_event hrm_raw_input;

	while ((syn == false) && (read_input_cnt < INPUT_MAX_BEFORE_SYN)) {
		int len = read(handle, &hrm_raw_input, sizeof(hrm_raw_input));
		if (len != sizeof(hrm_raw_input)) {
			_E("hrm_raw_file read fail, read_len = %d", len);
			return false;
		}

		++read_input_cnt;

		if (hrm_raw_input.type == EV_REL) {
			index = hrm_raw_input.code - REL_X;

			/* Check an avaiable value REL_X(0x00) ~ REL_MISC(0x09) */
			if (index >= INPUT_VALUE_COUNT) {
				_E("hrm_raw_input event[type = %d, code = %d] is unknown.", hrm_raw_input.type, index);
				return false;
			}
			data.values[index] = (unsigned int)hrm_raw_input.value - INPUT_EVENT_BIAS;

		} else if (hrm_raw_input.type == EV_SYN) {
			syn = true;
			data.timestamp = util::get_timestamp(&hrm_raw_input.time);
			data.value_count = INPUT_VALUE_COUNT;
		} else {
			_E("hrm_raw_input event[type = %d, code = %d] is unknown.", hrm_raw_input.type, hrm_raw_input.code);
			return false;
		}
	}

	if (!syn) {
		_E("EV_SYN didn't come until %d inputs had come", read_input_cnt);
		return false;
	}
	return true;
}
