add_executable(station weather_station.cpp)
ament_target_dependencies(station 
    rclcpp 
    std_msgs 
    sensor_msgs
)

add_executable(listener weather_listener.cpp)
ament_target_dependencies(listener 
    rclcpp 
    std_msgs 
    sensor_msgs
)

install(TARGETS
    station
    listener
    DESTINATION lib/${PROJECT_NAME}
)
