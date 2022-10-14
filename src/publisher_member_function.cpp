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
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/temperature.hpp"
#include "sensor_msgs/msg/relative_humidity.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class WheatherStation : public rclcpp::Node
{
public:
  WheatherStation()
  : Node("WheatherStation") {
    temperature_ = this->create_publisher<sensor_msgs::msg::Temperature>("temperature", 10);
    humidity_ = this->create_publisher<sensor_msgs::msg::RelativeHumidity>("humidity", 10);
    timer_ = this->create_wall_timer(
      500ms, std::bind(&WheatherStation::timer_callback, this));
  }

private:
  void timer_callback() {
    auto temp = sensor_msgs::msg::Temperature();
    temp.temperature = 15.0;

    auto humty = sensor_msgs::msg::RelativeHumidity();
    humty.variance = 1.0;

    RCLCPP_INFO(this->get_logger(), "t: '%f' *C; h: '%f'", temp.temperature, humty.variance);
    temperature_->publish(temp);
    humidity_->publish(humty);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr temperature_;
  rclcpp::Publisher<sensor_msgs::msg::RelativeHumidity>::SharedPtr humidity_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WheatherStation>());
  rclcpp::shutdown();
  return 0;
}
