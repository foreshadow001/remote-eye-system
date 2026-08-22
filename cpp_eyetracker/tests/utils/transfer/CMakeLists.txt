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
