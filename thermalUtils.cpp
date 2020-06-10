/*
 * Copyright (c) 2020, The Linux Foundation. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *	* Redistributions of source code must retain the above copyright
 *	  notice, this list of conditions and the following disclaimer.
 *	* Redistributions in binary form must reproduce the above
 *	  copyright notice, this list of conditions and the following
 *	  disclaimer in the documentation and/or other materials provided
 *	  with the distribution.
 *	* Neither the name of The Linux Foundation nor the names of its
 *	  contributors may be used to endorse or promote products derived
 *	  from this software without specific prior written permission.
 *
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
 * IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <android-base/file.h>
#include <android-base/logging.h>
#include <android-base/properties.h>
#include <android-base/stringprintf.h>
#include <android-base/strings.h>
#include <hidl/HidlTransportSupport.h>

#include "thermalConfig.h"
#include "thermalUtils.h"

namespace android {
namespace hardware {
namespace thermal {
namespace V2_0 {
namespace implementation {

ThermalUtils::ThermalUtils(const ueventCB &inp_cb):
	cfg(),
	cmnInst(),
	monitor(std::bind(&ThermalUtils::ueventParse, this,
				std::placeholders::_1,
				std::placeholders::_2)),
	cb(inp_cb)
{
	int ret = 0;
	std::vector<struct therm_sensor> sensorList;
	std::vector<struct therm_sensor>::iterator it;

	is_sensor_init = false;
	is_cdev_init = false;
	ret = cmnInst.initThermalZones(cfg.fetchConfig());
	if (ret > 0) {
		is_sensor_init = true;
		sensorList = cmnInst.fetch_sensor_list();
		std::lock_guard<std::mutex> _lock(sens_cb_mutex);
		for (it = sensorList.begin(); it != sensorList.end(); it++) {
			thermalConfig[it->sensor_name] = *it;
			cmnInst.read_temperature(&(*it));
			cmnInst.initThreshold(*it);
		}
		monitor.start();
	}
	ret = cmnInst.initCdev();
	if (ret > 0) {
		is_cdev_init = true;
		cdevList = cmnInst.fetch_cdev_list();
	}
}

void ThermalUtils::ueventParse(std::string sensor_name, int temp)
{
	std::unordered_map<std::string, struct therm_sensor>::iterator it;
	struct therm_sensor sens;

	LOG(INFO) << "uevent triggered for sensor: " << sensor_name
		<< std::endl;
	it = thermalConfig.find(sensor_name);
	if (it == thermalConfig.end()) {
		LOG(DEBUG) << "sensor is not monitored:" << sensor_name
			<< std::endl;
		return;
	}
	sens = it->second;
	std::lock_guard<std::mutex> _lock(sens_cb_mutex);
	sens.t.value = (float)temp / (float)sens.mulFactor;
	cmnInst.estimateSeverity(&sens);
	if (sens.lastThrottleStatus != sens.t.throttlingStatus) {
		LOG(INFO) << "sensor: " << sensor_name <<" old: " <<
			(int)sens.lastThrottleStatus << " new: " <<
			(int)sens.t.throttlingStatus << std::endl;
		cb(sens.t);
		cmnInst.initThreshold(sens);
	}
}

int ThermalUtils::readTemperatures(hidl_vec<Temperature_1_0> *temp)
{
	std::unordered_map<std::string, struct therm_sensor>::iterator it;
	int ret = 0, idx = 0;

	if (!is_sensor_init)
		return 0;
	temp->resize(thermalConfig.size());
	for (it = thermalConfig.begin(); it != thermalConfig.end();
			it++, idx++) {
		struct therm_sensor sens = it->second;
		ret = cmnInst.read_temperature(&sens);
		if (ret < 0)
			return ret;
		(*temp)[idx].currentValue = sens.t.value;
		(*temp)[idx].name = sens.t.name;
		(*temp)[idx].type = (TemperatureType_1_0)sens.t.type;
	}

	return temp->size();
}

int ThermalUtils::readTemperatures(bool filterType, TemperatureType type,
                                            hidl_vec<Temperature> *temperatures)
{
	std::vector<Temperature> local_temp;
	std::unordered_map<std::string, struct therm_sensor>::iterator it;
	int ret = 0;
	Temperature nantemp;

	for (it = thermalConfig.begin(); it != thermalConfig.end(); it++) {
		struct therm_sensor sens = it->second;

		if (filterType && sens.t.type != type)
			continue;
		ret = cmnInst.read_temperature(&sens);
		if (ret < 0)
			return ret;
		local_temp.push_back(sens.t);
	}
	if (local_temp.empty()) {
		nantemp.type = type;
		nantemp.value = UNKNOWN_TEMPERATURE;
		local_temp.push_back(nantemp);
	}
	*temperatures = local_temp;

	return temperatures->size();
}

int ThermalUtils::readTemperatureThreshold(bool filterType, TemperatureType type,
                                            hidl_vec<TemperatureThreshold> *thresh)
{
	std::vector<TemperatureThreshold> local_thresh;
	std::unordered_map<std::string, struct therm_sensor>::iterator it;
	int idx = 0;
	TemperatureThreshold nanthresh;

	for (it = thermalConfig.begin(); it != thermalConfig.end(); it++) {
		struct therm_sensor sens = it->second;

		if (filterType && sens.t.type != type)
			continue;
		local_thresh.push_back(sens.thresh);
	}
	if (local_thresh.empty()) {
		nanthresh.type = type;
		nanthresh.vrThrottlingThreshold = UNKNOWN_TEMPERATURE;
		for (idx = 0; idx < (size_t)ThrottlingSeverity::SHUTDOWN;
				idx++) {
			nanthresh.hotThrottlingThresholds[idx] =
			nanthresh.coldThrottlingThresholds[idx] =
				UNKNOWN_TEMPERATURE;
		}
		local_thresh.push_back(nanthresh);
	}
	*thresh = local_thresh;

	return thresh->size();
}

int ThermalUtils::readCdevStates(bool filterType, cdevType type,
                                            hidl_vec<CoolingDevice> *cdev_out)
{
	std::vector<CoolingDevice> local_cdev;
	std::vector<struct therm_cdev>::iterator it;
	int ret = 0;
	CoolingDevice nanCdev;

	for (it = cdevList.begin(); it != cdevList.end(); it++) {
		struct therm_cdev cdev = *it;

		if (filterType && cdev.c.type != type)
			continue;
		ret = cmnInst.read_cdev_state(&cdev);
		if (ret < 0)
			return ret;
		local_cdev.push_back(cdev.c);
	}
	if (local_cdev.empty()) {
		nanCdev.type = type;
		nanCdev.value = UNKNOWN_TEMPERATURE;
		local_cdev.push_back(nanCdev);
	}
	*cdev_out = local_cdev;

	return cdev_out->size();
}

int ThermalUtils::fetchCpuUsages(hidl_vec<CpuUsage> *cpu_usages)
{
	return cmnInst.get_cpu_usages(cpu_usages);
}

}  // namespace implementation
}  // namespace V2_0
}  // namespace thermal
}  // namespace hardware
}  // namespace android
