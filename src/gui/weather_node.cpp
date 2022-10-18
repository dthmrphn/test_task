#include "weather_node.h"

QRosNode::QRosNode(QObject* parent, int argc, char* argv[]) : QObject(parent) {
    rclcpp::init(argc, argv);

    using executor = rclcpp::executors::StaticSingleThreadedExecutor;

    auto exec = std::make_shared<executor>();
    ros_node_ = rclcpp::Node::make_shared("Weather");
    exec->add_node(ros_node_);

    temperature_ = ros_node_->create_subscription<Temperature>(
        "temperature", 10,
        std::bind(&QRosNode::temperature_cb, this, std::placeholders::_1));

    relhumidity_ = ros_node_->create_subscription<RelHumidity>(
        "relhumidity", 10,
        std::bind(&QRosNode::relhumidity_cb, this, std::placeholders::_1));

    std::thread exec_thread(std::bind(
        &executor::spin, exec));
    exec_thread.detach();
}

void QRosNode::temperature_cb(const Temperature& msg) {
    RCLCPP_INFO(ros_node_->get_logger(), "t: '%f' *C", msg.temperature);
    count_tmp++;
    accumulator_tmp += msg.temperature;
    emit temperatureReceived(accumulator_tmp / count_tmp);
}

void QRosNode::relhumidity_cb(const RelHumidity& msg) {
    RCLCPP_INFO(ros_node_->get_logger(), "h: '%f'", msg.relative_humidity);
    count_hmt++;
    accumulator_hmt += msg.relative_humidity;
    emit relhumidityReceived(accumulator_hmt / count_hmt);
}


