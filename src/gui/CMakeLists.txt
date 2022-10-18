set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

list(REMOVE_DUPLICATES QML_IMPORT_PATH)
set(QML_IMPORT_PATH ${QML_IMPORT_PATH}
    CACHE STRING "extra qml import paths"
    FORCE
)

add_executable(stationgui 
    qml/qml.qrc 
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
