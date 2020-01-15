/**
 * @brief System Monitor ROS 2 node
 * @file system_monitor_node.cpp
 * @addtogroup nodes
 * @author Nuno Marques <nuno.marques@dronesolutions.io>
 * @author Tanja Baumann <tanja@auterion.com
 * @date Jan 15, 2020
 *
 *      launch example:
 *      ros2 launch system_monitor_ros system_monitor.launch.py
 */

#include <chrono>
#include <rclcpp/rclcpp.hpp>

#include <px4_msgs/msg/onboard_computer_status.hpp>
#include <std_msgs/msg/string.hpp>

#include <system_monitor_ros/system_monitor.h>

using namespace std::chrono_literals;

class OnboardComputerStatusPublisher : public rclcpp::Node {
 public:
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

  OnboardComputerStatusPublisher() : Node("system_monitor_node") {
    ocs_publisher_ =
        this->create_publisher<px4_msgs::msg::OnboardComputerStatus>("OnboardComputerStatus_PubSubTopic", 1);
    cpu_publisher_ = this->create_publisher<std_msgs::msg::String>("/cpu_usage", 1);
    memory_pub_ = this->create_publisher<std_msgs::msg::String>("/memory_usage", 1);
    processes_pub_ = this->create_publisher<std_msgs::msg::String>("/processes", 1);

    int n_processes = this->declare_parameter("n_processes", 8);
    RCLCPP_DEBUG(this->get_logger(), "n_processes: %d", n_processes);

    auto timer_callback = [this, n_processes]() -> void {
      // Get CPU and Memory usage
      std::string cpu = exec(" top -bn 1 | sed -n 3p");
      std::string mem = exec(" top -bn 1 | sed -n 4p");

      // Get n_processes most CPU-hungry processes
      std::string command = "ps -e -o pid,pcpu,pmem,args --sort=-pcpu |  head -n ";
      command.append(std::to_string(n_processes + 1));
      command.append(" | cut -d' ' -f1-8");
      std::string processes = exec(command.c_str());

      // Create messages
      auto status_msg = px4_msgs::msg::OnboardComputerStatus();

      status_msg.timestamp = this->now().nanoseconds() * 1E-3;
      status_msg.cpu_cores = system_monitor_->getCpuCores();
      status_msg.cpu_combined = system_monitor_->getCpuCombined();
      status_msg.temperature_board = system_monitor_->getBoardTemperature();
      status_msg.ram_usage = system_monitor_->getRamUsage();
      status_msg.ram_total = system_monitor_->getRamTotal();
      status_msg.uptime = system_monitor_->getUpTime();

      auto cpu_msg = std_msgs::msg::String();
      cpu_msg.data = cpu;

      auto mem_msg = std_msgs::msg::String();
      mem_msg.data = mem;

      auto proc_msg = std_msgs::msg::String();
      proc_msg.data = processes;

      // Publish messages
      this->ocs_publisher_->publish(status_msg);
      this->cpu_publisher_->publish(cpu_msg);
      this->memory_pub_->publish(mem_msg);
      this->processes_pub_->publish(proc_msg);
    };
    system_monitor_ = std::make_shared<SystemMonitor>();
    timer_ = this->create_wall_timer(500ms, timer_callback);
  }

 private:
  std::shared_ptr<SystemMonitor> system_monitor_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<px4_msgs::msg::OnboardComputerStatus>::SharedPtr ocs_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr cpu_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr memory_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr processes_pub_;
};

int main(int argc, char* argv[]) {
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OnboardComputerStatusPublisher>());

  rclcpp::shutdown();
  return 0;
}
