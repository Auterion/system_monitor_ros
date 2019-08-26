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

std::vector<CpuData> SystemMonitor::GetCpuTimes() {
    std::vector<CpuData> entries;
 	const std::string STR_CPU("cpu");
	const std::size_t LEN_STR_CPU = STR_CPU.size();
	const std::string STR_TOT("total");

    std::string line;

    std::ifstream proc_stat("/proc/stat");

    while(std::getline(proc_stat, line)){
        if(!line.compare(0, LEN_STR_CPU, STR_CPU)) {
        	std::istringstream ss(line);

			// store entry
			entries.emplace_back(CpuData());
			CpuData & entry = entries.back();

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

size_t SystemMonitor::GetIdleTime(const CpuData & e) {
    return  e.times[CpuStates::IDLE] +
            e.times[CpuStates::IOWAIT];
}

size_t SystemMonitor::GetActiveTime(const CpuData & e) {
    return  e.times[CpuStates::USER] +
            e.times[CpuStates::NICE] +
            e.times[CpuStates::SYSTEM] +
            e.times[CpuStates::IRQ] +
            e.times[CpuStates::SOFTIRQ] +
            e.times[CpuStates::STEAL] +
            e.times[CpuStates::GUEST] +
            e.times[CpuStates::GUEST_NICE];
}

void SystemMonitor::readCpuUsage(){
    std::vector<CpuData> cpu_times = GetCpuTimes();

	if(prev_cpu_times_.size() == 0)	prev_cpu_times_ = cpu_times; //Handle exception

	for(size_t i = 0; i < cpu_times.size(); ++i)	{
		const CpuData & e1 = prev_cpu_times_[i];
		const CpuData & e2 = cpu_times[i];

		const float active_time	= static_cast<float>(GetActiveTime(e2) - GetActiveTime(e1));
		const float idle_time	= static_cast<float>(GetIdleTime(e2) - GetIdleTime(e1));
		const float total_time	= active_time + idle_time;

		if(i == 0 )	cpu_combined_[0] = uint8_t(100.f * active_time / total_time);
		else	cpu_cores_[i-1] = uint8_t(100.f * active_time / total_time);
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