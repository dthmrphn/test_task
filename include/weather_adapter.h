#ifndef WEATHER_ADAPTER_H_
#define WEATHER_ADAPTER_H_

#include "rclcpp/type_adapter.hpp"
#include "sensor_msgs/msg/relative_humidity.hpp"
#include "sensor_msgs/msg/temperature.hpp"

using Temperature = sensor_msgs::msg::Temperature;
using RelHumidity = sensor_msgs::msg::RelativeHumidity;

template <> struct rclcpp::TypeAdapter<float, Temperature> {
    using is_specialized = std::true_type;
    using custom_type = float;
    using ros_message_type = Temperature;

    static void convert_to_ros_message(const custom_type& src,
                                       ros_message_type& dst) {
        dst.temperature = src;
    }

    static void convert_to_custom(const ros_message_type& src,
                                  custom_type& dst) {
        dst = src.temperature;
    }
};

template <> struct rclcpp::TypeAdapter<float, RelHumidity> {
    using is_specialized = std::true_type;
    using custom_type = float;
    using ros_message_type = RelHumidity;

    static void convert_to_ros_message(const custom_type& src,
                                       ros_message_type& dst) {
        dst.relative_humidity = src;
    }

    static void convert_to_custom(const ros_message_type& src,
                                  custom_type& dst) {
        dst = src.relative_humidity;
    }
};

#endif
