/*
 * system_monitor_node.cpp
 *
 *  Created on: Dez 05, 2018
 *      Author: Tanja Baumann
 *
 *      launch example:
 *      roslaunch system_monitor_ros n_processes:=10
 */

#include <mavros_msgs/OnboardComputerStatus.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <fstream>
#include <iostream>

#include <array>
#include <cstdio>
#include <memory>
#include <stdexcept>
#include <string>

#include <system_monitor_ros/system_monitor.h>

std::string exec(const char* cmd) {
  std::array<char, 128> buffer;
  std::string result;
  std::shared_ptr<FILE> pipe(popen(cmd, "r"), pclose);
  if (!pipe) throw std::runtime_error("popen() failed!");
  while (!feof(pipe.get())) {
    if (fgets(buffer.data(), 128, pipe.get()) != nullptr) result += buffer.data();
  }
  return result;
}

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "system_monitor_node");
  ros::NodeHandle nh("~");

  // get parameter
  int n_processes;
  nh.param<int>("n_processes", n_processes, 8);

  // initialize publishers
  ros::Publisher cpu_pub_;
  ros::Publisher memory_pub_;
  ros::Publisher processes_pub_;
  ros::Publisher onboardstatus_pub_;

  ros::Timer loop_timer_;

  SystemMonitor system_monitor_;

  cpu_pub_ = nh.advertise<std_msgs::String>("/cpu_usage", 1);
  memory_pub_ = nh.advertise<std_msgs::String>("/memory_usage", 1);
  processes_pub_ = nh.advertise<std_msgs::String>("/processes", 1);
  onboardstatus_pub_ = nh.advertise<mavros_msgs::OnboardComputerStatus>("/mavros/onboard_computer/status", 1);

  while (ros::ok()) {
    // define messages
    std_msgs::String cpu_msg;
    std_msgs::String mem_msg;
    std_msgs::String proc_msg;
    mavros_msgs::OnboardComputerStatus status_msg;

    // get cpu and memory usage
    std::string cpu = exec(" top -bn 1 | sed -n 3p");
    std::string mem = exec(" top -bn 1 | sed -n 4p");

    // Get n_processes most CPU-hungry processes
    std::string command = "ps -e -o pid,pcpu,pmem,args --sort=-pcpu |  head -n ";
    command.append(std::to_string(n_processes + 1));
    command.append(" | cut -d' ' -f1-8");
    std::string processes = exec(command.c_str());

    cpu_msg.data = cpu;
    mem_msg.data = mem;
    proc_msg.data = processes;

    // Assemble onboard status messages
    status_msg.header.stamp = ros::Time::now();
    status_msg.cpu_cores = system_monitor_.getCpuCores();
    status_msg.cpu_combined = system_monitor_.getCpuCombined();
    status_msg.temperature_board = system_monitor_.getBoardTemperature();
    status_msg.uptime = system_monitor_.getUpTime();

    // publish system data
    onboardstatus_pub_.publish(status_msg);
    cpu_pub_.publish(cpu_msg);
    memory_pub_.publish(mem_msg);
    processes_pub_.publish(proc_msg);

    ros::Duration(1).sleep();
  }

  return 0;
}
