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

#include <sensor/sensor_hal.h>
#include <sensor_log.h>
#include <vector>

#include "accel/accel_device.h"
#include "gyro/gyro_device.h"
#include "geomag/geomag_device.h"
#include "pressure/pressure_device.h"
#include "ultraviolet/uv_device.h"
#include "light/light_device.h"
#include "proxi/proxi_device.h"
#include "hrm/hrm_device.h"

static std::vector<sensor_device_t> devs;

template<typename _sensor>
void create_sensor(const char *name)
{
	sensor_device *instance = NULL;
	try {
		instance = new _sensor;
	} catch (std::exception &e) {
		ERR("Failed to create %s sensor device, exception: %s", name, e.what());
		return;
	} catch (int err) {
		_ERRNO(err, _E, "Failed to create %s sensor device", name);
		return;
	}

	devs.push_back(instance);
}

extern "C" int create(sensor_device_t **devices)
{
	create_sensor<accel_device>("Accelerometer");
	create_sensor<gyro_device>("Gyroscope");
	create_sensor<geomag_device>("Geomagnetic");
	create_sensor<pressure_device>("Pressure");
	create_sensor<uv_device>("Ultra Violet");
	create_sensor<light_device>("Light");
	create_sensor<proxi_device>("Proximity");
	create_sensor<hrm_device>("HRM");

	*devices = &devs[0];
	return devs.size();
}
