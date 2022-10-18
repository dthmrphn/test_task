set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

add_executable(stationgui 
    qml/main.qml 
    main.cpp
    weather_node.cpp
)

ament_target_dependencies(stationgui 
    rclcpp 
    std_msgs 
    sensor_msgs
)

target_link_libraries(stationgui 
    Qt5::Core 
    Qt5::Quick
)

install(TARGETS
    stationgui
    DESTINATION lib/${PROJECT_NAME}
)

# install(FILES
#     qml/main.qml
#     DESTINATION lib/${PROJECT_NAME}
# )
