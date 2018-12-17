/*
 * system_monitor_node.cpp
 *
 *  Created on: Dez 05, 2018
 *      Author: Tanja Baumann
 *
 *      launch example:
 *      roslaunch system_monitor_ros n_processes:=10
 */

#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <std_msgs/String.h>

#include <cstdio>
#include <memory>
#include <stdexcept>
#include <string>
#include <array>


std::string exec(const char* cmd) {
    std::array<char, 128> buffer;
    std::string result;
    std::shared_ptr<FILE> pipe(popen(cmd, "r"), pclose);
    if (!pipe) throw std::runtime_error("popen() failed!");
    while (!feof(pipe.get())) {
        if (fgets(buffer.data(), 128, pipe.get()) != nullptr)
            result += buffer.data();
    }
    return result;
}


int main(int argc, char* argv[])
{
	ros::init(argc, argv, "system_monitor_node");
	ros::NodeHandle nh("~");

	//get parameter
	int n_processes, n_cores;
	nh.param<int>("n_processes", n_processes, 8);
        nh.param<int>("n_cores", n_cores, 8);

	//initialize publishers
	ros::Publisher cpu_pub_;
	ros::Publisher memory_pub_;
	ros::Publisher processes_pub_;

	cpu_pub_ = nh.advertise<std_msgs::String>("/cpu_usage", 1);
	memory_pub_ = nh.advertise<std_msgs::String>("/memory_usage", 1);
	processes_pub_ = nh.advertise<std_msgs::String>("/processes", 1);


	while (ros::ok()) {

	    // define messages
	    std_msgs::String cpu_msg;
	    std_msgs::String mem_msg;
	    std_msgs::String proc_msg;

	    //get cpu and memory usage
	    std::string cpu_request_cmd = std::string(" top -bn 1 | awk '{print $9}' | tail -n +8 | awk '{s+=$1} END {print s/") + std::to_string(n_cores) + std::string("}'");
            std::string cpu = exec(cpu_request_cmd.c_str());
	    std::string mem = exec(" top -b -n1 | grep Mem | head -1 | awk '{print $4, $5, $6, $7, $8, $9, $10, $11}'");

	    //Get n_processes most CPU-hungry processes
	    std::string command = "ps -e -o pid,pcpu,pmem,args --sort=-pcpu |  head -n ";
	    command.append(std::to_string(n_processes + 1));
	    command.append(" | cut -d' ' -f1-8");
	    std::string processes = exec(command.c_str());

	    cpu_msg.data = cpu;
	    mem_msg.data = mem;
	    proc_msg.data = processes;

	    //publish system data
	    cpu_pub_.publish(cpu_msg);
	    memory_pub_.publish(mem_msg);
	    processes_pub_.publish(proc_msg);

		ros::Duration(1).sleep();
	}


    return 0;
}

