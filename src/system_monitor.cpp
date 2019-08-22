/*
 * system_monitor.cpp
 *
 *  Created on: Aug 21, 2019
 *      Author: Jaeyoung Lim
 *
 */

#include <fstream>
#include <iostream>
#include <numeric>
#include <unistd.h>
#include <sstream>

#include <system_monitor_ros/system_monitor.h>

SystemMonitor::SystemMonitor(){

}

SystemMonitor::~SystemMonitor(){

}

std::vector<CPUData> SystemMonitor::GetCpuTimes() {
    std::vector<CPUData> entries;
 	const std::string STR_CPU("cpu");
	const std::size_t LEN_STR_CPU = STR_CPU.size();
	const std::string STR_TOT("tot");

    std::string line;

    std::ifstream proc_stat("/proc/stat");

    while(std::getline(proc_stat, line)){
        if(!line.compare(0, LEN_STR_CPU, STR_CPU)) {
        	std::istringstream ss(line);

			// store entry
			entries.emplace_back(CPUData());
			CPUData & entry = entries.back();

			// read cpu label
			ss >> entry.cpu;

			if(entry.cpu.size() > LEN_STR_CPU)
				entry.cpu.erase(0, LEN_STR_CPU);
			else
				entry.cpu = STR_TOT;

			// read times
			for(int i = 0; i < NUM_CPU_STATES; ++i)
				ss >> entry.times[i];
		}
    }
    return entries;

}

size_t SystemMonitor::GetIdleTime(const CPUData & e) {
    return  e.times[S_IDLE] +
            e.times[S_IOWAIT];
}

size_t SystemMonitor::GetActiveTime(const CPUData & e) {
    return  e.times[S_USER] +
            e.times[S_NICE] +
            e.times[S_SYSTEM] +
            e.times[S_IRQ] +
            e.times[S_SOFTIRQ] +
            e.times[S_STEAL] +
            e.times[S_GUEST] +
            e.times[S_GUEST_NICE];
}

void SystemMonitor::readCpuUsage(){
    std::vector<CPUData> cpu_times = GetCpuTimes();
	int NUM_ENTRIES = cpu_times.size();

	if(prev_cpu_times_.size() == 0)	prev_cpu_times_ = cpu_times;

	for(size_t i = 0; i < NUM_ENTRIES; ++i)
	{
		const CPUData & e1 = prev_cpu_times_[i];
		const CPUData & e2 = cpu_times[i];

		const float ACTIVE_TIME	= static_cast<float>(GetActiveTime(e2) - GetActiveTime(e1));
		const float IDLE_TIME	= static_cast<float>(GetIdleTime(e2) - GetIdleTime(e1));
		const float TOTAL_TIME	= ACTIVE_TIME + IDLE_TIME;

		if(i == 0 ){
			//TODO: Shift cpu_combined message
			cpu_combined_[0] = uint8_t(100.f * ACTIVE_TIME / TOTAL_TIME);
		} else {
			cpu_cores_[i-1] = uint8_t(100.f * ACTIVE_TIME / TOTAL_TIME);
		}
	}
	prev_cpu_times_ = cpu_times;
}
boost::array<uint8_t, 8> SystemMonitor::getCpuCores() {
	readCpuUsage();
	return cpu_cores_;
}

boost::array<uint8_t, 10> SystemMonitor::getCpuCombined(){
	return cpu_combined_;
}