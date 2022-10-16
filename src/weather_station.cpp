#include <chrono>
#include <fstream>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/node_interfaces/node_parameters_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/relative_humidity.hpp"
#include "sensor_msgs/msg/temperature.hpp"

using Temperature = sensor_msgs::msg::Temperature;
using RelHumidity = sensor_msgs::msg::RelativeHumidity;

template <typename T> T get_message(float value) {}

template <> Temperature get_message<Temperature>(float value) {
    Temperature rv;
    rv.temperature = value;
    return rv;
}

template <> RelHumidity get_message<RelHumidity>(float value) {
    RelHumidity rv;
    rv.variance = value;
    return rv;
}

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
        publisher_ = this->create_publisher<T>(n, 10);

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
        auto value = read_value(f_);
        T message = get_message<T>(value);
        RCLCPP_INFO(this->get_logger(), "v: '%f'", value);
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
    std::shared_ptr<rclcpp::Publisher<T>> publisher_;

    static constexpr std::string_view source = "source";
    static constexpr std::string_view period = "period";

    std::fstream f_;
    int period_ = 500;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    rclcpp::executors::SingleThreadedExecutor exec;

    auto temp = std::make_shared<WeatherSensor<Temperature>>(
        "temperature",
        "/home/deth/ros2_ws/src/test_task/measures/temperature.txt");
    auto humy = std::make_shared<WeatherSensor<RelHumidity>>(
        "humidity", "/home/deth/ros2_ws/src/test_task/measures/humidity.txt");

    exec.add_node(temp);
    exec.add_node(humy);
    exec.spin();

    rclcpp::shutdown();
    return 0;
}
