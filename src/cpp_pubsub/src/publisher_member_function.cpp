// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <memory>
#include <fstream>
#include <iostream>
#include <numeric>
#include <unistd.h>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
// #include "cpu_load.h"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node
{
public:
  size_t previous_idle_time=0; 
  size_t previous_total_time=0;
  MinimalPublisher()
  : Node("minimal_publisher"), count_(0)
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    timer_ = this->create_wall_timer(
      5000ms, std::bind(&MinimalPublisher::timer_callback, this));
  }
private:
  bool get_cpu_times(size_t &idle_time, size_t &total_time) {
    std::ifstream proc_stat("/proc/stat");
    proc_stat.ignore(5, ' '); // Skip the 'cpu' prefix.
    std::vector<size_t> cpu_times;
    for (size_t time; proc_stat >> time; cpu_times.push_back(time));
    if (cpu_times.size() < 4)
        return false;
    idle_time = cpu_times[3];
    total_time = std::accumulate(cpu_times.begin(), cpu_times.end(), 0);
    return true;
  }
  
  private:
  void write_to_file(std::string data){
    std::ofstream outfile;

    outfile.open("./cpu_load.log", std::ios_base::app); // open file in append mode
    outfile << data << std::endl; 
    outfile.close();
  }

private:
  void timer_callback()
  {
    auto message = std_msgs::msg::String();
    size_t idle_time=0, total_time=0;
    float utilization = 0.0;
    // std::cout << idle_time << std::endl;
    if(get_cpu_times(idle_time, total_time)){
      const float idle_time_delta = idle_time - previous_idle_time;
      const float total_time_delta = total_time - previous_total_time;
      utilization = 100.0 * (1.0 - idle_time_delta / total_time_delta);
      previous_idle_time = idle_time;
      previous_total_time = total_time;
    }
    message.data = "CPU Utilization: " + std::to_string(utilization) + " %";
    write_to_file(std::to_string(utilization));
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
