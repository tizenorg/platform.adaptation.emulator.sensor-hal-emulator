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

#ifdef ENABLE_ACCEL
#include "accel/accel_device.h"
#endif
#ifdef ENABLE_GYRO
#include "gyro/gyro_device.h"
#endif
#ifdef ENABLE_GYRO_UNCAL
#include "gyro_uncal/gyro_uncal_device.h"
#endif
#ifdef ENABLE_GEOMAG
#include "geomag/geomag_device.h"
#endif
#ifdef ENABLE_PRESSURE
#include "pressure/pressure_device.h"
#endif
#ifdef ENABLE_UV
#include "ultraviolet/uv_device.h"
#endif
#ifdef ENABLE_LIGHT
#include "light/light_device.h"
#endif
#ifdef ENABLE_PROXIMITY
#include "proxi/proxi_device.h"
#endif
#ifdef ENABLE_HRM_RAW
#include "hrm_raw/hrm_raw_device.h"
#endif
#ifdef ENABLE_HRM
#include "hrm/hrm_device.h"
#endif
#ifdef ENABLE_SENSORHUB
#include "sensorhub/sensorhub.h"
#endif

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
#ifdef ENABLE_ACCEL
	create_sensor<accel_device>("Accelerometer");
#endif
#ifdef ENABLE_GYRO
	create_sensor<gyro_device>("Gyroscope");
#endif
#ifdef ENABLE_GYRO_UNCAL
	create_sensor<gyro_uncal_device>("Gyroscope Uncal");
#endif
#ifdef ENABLE_GEOMAG
	create_sensor<geomag_device>("Geomagnetic");
#endif
#ifdef ENABLE_PRESSURE
	create_sensor<pressure_device>("Pressure");
#endif
#ifdef ENABLE_UV
	create_sensor<uv_device>("Ultra Violet");
#endif
#ifdef ENABLE_LIGHT
	create_sensor<light_device>("Light");
#endif
#ifdef ENABLE_PROXIMITY
	create_sensor<proxi_device>("Proximity");
#endif
#ifdef ENABLE_HRM_RAW
	create_sensor<hrm_raw_device>("HRM Raw");
#endif
#ifdef ENABLE_HRM
	create_sensor<hrm_device>("HRM");
#endif
#ifdef ENABLE_SENSORHUB
	create_sensor<sensorhub_device>("Sensorhub");
#endif

	*devices = &devs[0];
	return devs.size();
}
