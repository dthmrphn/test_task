#ifndef ROS_NODE_H_
#define ROS_NODE_H_

#include <functional>
#include <memory>

#include <QObject>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/relative_humidity.hpp"
#include "sensor_msgs/msg/temperature.hpp"

#include "weather_adapter.h"

class QRosNode : public QObject {
    Q_OBJECT
  public:
    explicit QRosNode(QObject* parent = nullptr, int argc = 0, char* argv[] = nullptr);

  private:
    void temperature_cb(const Temperature& msg);
    void relhumidity_cb(const RelHumidity& msg);

    float accumulator_tmp = 0;
    float accumulator_hmt = 0;
    size_t count_tmp = 0;
    size_t count_hmt = 0;

    rclcpp::Node::SharedPtr ros_node_;
    rclcpp::Subscription<Temperature>::SharedPtr temperature_;
    rclcpp::Subscription<RelHumidity>::SharedPtr relhumidity_;

  signals:
    void temperatureReceived(float value);
    void relhumidityReceived(float value);
};

#endif
