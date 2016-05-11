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

#ifndef _SENSOR_LOG_H_
#define _SENSOR_LOG_H_

#include <dlog.h>

#ifndef NAME_MAX
#define NAME_MAX 256
#endif

#define EVENT_TYPE_SHIFT 16
#define SENSOR_TYPE_SHIFT 32

#ifdef LOG_TAG
	#undef LOG_TAG
#endif
#define LOG_TAG	"SENSOR"

#define LOG_DUMP(fp, fmt, arg...) do { if (fp) fprintf(fp, fmt, ##arg); else _E(fmt, ##arg); } while(0)

#ifdef _DEBUG
#define DBG SLOGD
#else
#define DBG(...) do{} while(0)
#endif

#define ERR SLOGE
#define WARN SLOGW
#define INFO SLOGI
#define _E ERR
#define _W WARN
#define _I INFO
#define _D DBG

#define _ERRNO(errno, tag, fmt, arg...) do { \
		char buf[1024]; \
		char *error = strerror_r(errno, buf, 1024); \
		if (!error) { \
			_E("Failed to strerror_r()"); \
			break; \
		} \
		tag(fmt" (%s[%d])", ##arg, error, errno); \
	} while (0)

#ifdef _DEBUG
#define warn_if(expr, fmt, arg...) do { \
		if(expr) { \
			_D("(%s) -> " fmt, #expr, ##arg); \
		} \
	} while (0)
#define ret_if(expr) do { \
		if(expr) { \
			_D("(%s) -> %s() return", #expr, __FUNCTION__); \
			return; \
		} \
	} while (0)
#define retv_if(expr, val) do { \
		if(expr) { \
			_D("(%s) -> %s() return", #expr, __FUNCTION__); \
			return (val); \
		} \
	} while (0)
#define retm_if(expr, fmt, arg...) do { \
		if(expr) { \
			_E(fmt, ##arg); \
			_D("(%s) -> %s() return", #expr, __FUNCTION__); \
			return; \
		} \
	} while (0)
#define retvm_if(expr, val, fmt, arg...) do { \
		if(expr) { \
			_E(fmt, ##arg); \
			_D("(%s) -> %s() return", #expr, __FUNCTION__); \
			return (val); \
		} \
	} while (0)

#else
#define warn_if(expr, fmt, arg...) do { \
		if(expr) { \
			_E(fmt, ##arg); \
		} \
	} while (0)
#define ret_if(expr) do { \
		if(expr) { \
			return; \
		} \
	} while (0)
#define retv_if(expr, val) do { \
		if(expr) { \
			return (val); \
		} \
	} while (0)
#define retm_if(expr, fmt, arg...) do { \
		if(expr) { \
			_E(fmt, ##arg); \
			return; \
		} \
	} while (0)
#define retvm_if(expr, val, fmt, arg...) do { \
		if(expr) { \
			_E(fmt, ##arg); \
			return (val); \
		} \
	} while (0)

#endif

#endif /* _SENSOR_LOG_H_ */
