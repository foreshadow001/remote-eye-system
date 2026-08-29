# tests/utils/transfer/CMakeLists.txt

find_package(OpenCV CONFIG REQUIRED)

# send_data: 采集端 100G 发送器 (master/slave 握手 + 串行传输, C++)
add_executable(send_data
    send_data.cpp
)

target_link_libraries(send_data
    PRIVATE
        eyetracker::cfg
        ${OpenCV_LIBS}
)

message(STATUS "send_data will be built")

# send_slave: slave 独立发送器 (阶段一: 无 master 握手, 直传处理主机)
add_executable(send_slave
    send_slave.cpp
)

target_link_libraries(send_slave
    PRIVATE
        eyetracker::cfg
        ${OpenCV_LIBS}
)

message(STATUS "send_slave will be built")
