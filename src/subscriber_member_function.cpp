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

#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/temperature.hpp"
#include "sensor_msgs/msg/relative_humidity.hpp"

using std::placeholders::_1;

class WheatherSubscriber : public rclcpp::Node
{
public:
  WheatherSubscriber()
  : Node("WheatherSubscriber") {
    temperature_ = this->create_subscription<sensor_msgs::msg::Temperature>(
      "temperature", 10, std::bind(&WheatherSubscriber::temp_callback, this, _1));
    humidity_ = this->create_subscription<sensor_msgs::msg::RelativeHumidity>(
      "humidity", 10, std::bind(&WheatherSubscriber::humi_callback, this, _1));
  }

private:
  void temp_callback(const sensor_msgs::msg::Temperature& msg) const
  {
    RCLCPP_INFO(this->get_logger(), "t: '%f' *C", msg.temperature);
  }
  void humi_callback(const sensor_msgs::msg::RelativeHumidity& msg) const
  {
    RCLCPP_INFO(this->get_logger(), "h: '%f'", msg.variance);
  }
  rclcpp::Subscription<sensor_msgs::msg::Temperature>::SharedPtr temperature_;
  rclcpp::Subscription<sensor_msgs::msg::RelativeHumidity>::SharedPtr humidity_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WheatherSubscriber>());
  rclcpp::shutdown();
  return 0;
}
