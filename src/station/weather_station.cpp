#include <chrono>
#include <fstream>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/node_interfaces/node_parameters_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/relative_humidity.hpp"
#include "sensor_msgs/msg/temperature.hpp"

#include "weather_adapter.h"

// function reading values from file
static float read_value(std::fstream& f) {
    std::string s;
    if (f.eof()) {
        f.clear();
        f.seekg(0);
    }
    std::getline(f, s);
    return std::atof(s.c_str());
}

// class template for sensor stub
template <typename T> class WeatherSensor : public rclcpp::Node {
  public:
    WeatherSensor() {}
    WeatherSensor(const std::string& n, const std::string& s)
        : Node(n), f_(std::fstream(s)) {
        // declaring node params for source and period
        this->declare_parameter<std::string>(source.data(), s);
        this->declare_parameter<int>(period.data(), period_);

        // creating publisher
        publisher_ =
            this->create_publisher<rclcpp::TypeAdapter<float, T>>(n, 10);

        // registring timer callback
        timer_handle_ =
            this->create_wall_timer(std::chrono::milliseconds(period_),
                                    std::bind(&WeatherSensor::timer_cb, this));

        // registring callback on paramerters change
        param_handle_ = this->add_on_set_parameters_callback(
            std::bind(&WeatherSensor::param_cb, this, std::placeholders::_1));
    }

  private:
    // timer callback sends measured data to subscriber
    void timer_cb() {
        auto message = read_value(f_);
        RCLCPP_INFO(this->get_logger(), "v: '%f'", message);
        publisher_->publish(message);
    }

    // params callback changes params on sensor
    auto param_cb(const std::vector<rclcpp::Parameter>& params) {
        rcl_interfaces::msg::SetParametersResult rv;
        for (const auto& param : params) {
            if (param.get_name() == source) {
                f_ = std::fstream(param.as_string());
            }
            if (param.get_name() == period) {
                reset_timer(param.as_int());
            }
            rv.reason = "setting " + param.get_name() + " to " +
                        param.value_to_string();
        }

        rv.successful = true;
        return rv;
    }

    // resets timer and creates nwe with new params
    void reset_timer(int period) {
        timer_handle_->reset();
        timer_handle_ =
            this->create_wall_timer(std::chrono::milliseconds(period),
                                    std::bind(&WeatherSensor::timer_cb, this));
    }

  private:
    std::shared_ptr<rclcpp::TimerBase> timer_handle_;
    std::shared_ptr<OnSetParametersCallbackHandle> param_handle_;
    std::shared_ptr<rclcpp::Publisher<rclcpp::TypeAdapter<float, T>>>
        publisher_;

    static constexpr std::string_view source = "source";
    static constexpr std::string_view period = "period";

    std::fstream f_;
    int period_ = 500;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    std::string home = getenv("HOME");
    rclcpp::executors::SingleThreadedExecutor exec;

    auto temp = std::make_shared<WeatherSensor<Temperature>>(
        "temperature",
        home + "/ros2_ws/src/test_task/measures/temperature.txt");
    auto humy = std::make_shared<WeatherSensor<RelHumidity>>(
        "relhumidity",
        home + "/ros2_ws/src/test_task/measures/relhumidity.txt");

    exec.add_node(temp);
    exec.add_node(humy);
    exec.spin();

    rclcpp::shutdown();
    return 0;
}
