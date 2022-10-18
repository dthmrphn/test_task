#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/relative_humidity.hpp"
#include "sensor_msgs/msg/temperature.hpp"

#include "weather_adapter.h"

using std::placeholders::_1;

class WheatherSubscriber : public rclcpp::Node {
  public:
    WheatherSubscriber() : Node("WheatherSubscriber") {
        temperature_ = this->create_subscription<Temperature>(
            "temperature", 10,
            std::bind(&WheatherSubscriber::temp_callback, this, _1));
        humidity_ = this->create_subscription<RelHumidity>(
            "relhumidity", 10,
            std::bind(&WheatherSubscriber::humi_callback, this, _1));
    }

  private:
    void temp_callback(const Temperature& msg) const {
        RCLCPP_INFO(this->get_logger(), "t: '%f' *C", msg.temperature);
    }
    void humi_callback(const RelHumidity& msg) const {
        RCLCPP_INFO(this->get_logger(), "h: '%f'", msg.variance);
    }
    rclcpp::Subscription<Temperature>::SharedPtr temperature_;
    rclcpp::Subscription<RelHumidity>::SharedPtr humidity_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WheatherSubscriber>());
    rclcpp::shutdown();
    return 0;
}
