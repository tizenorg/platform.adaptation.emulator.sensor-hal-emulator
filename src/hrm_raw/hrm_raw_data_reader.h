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
#ifndef _HRM_RAW_DATA_READER_H_
#define _HRM_RAW_DATA_READER_H_

#include <sys/time.h>
#include <sensor/sensor_hal.h>

class hrm_raw_data_reader {
public:
	hrm_raw_data_reader();
	virtual ~hrm_raw_data_reader();

	virtual bool open(void);
	virtual bool close(void);
	virtual bool start(void);
	virtual bool stop(void);
	virtual bool get_data(int handle, sensor_data_t &data) = 0;
};
#endif /* _HRM_RAW_DATA_READER_ */
