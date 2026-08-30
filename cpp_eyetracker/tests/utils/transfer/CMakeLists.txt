# tests/utils/transfer/CMakeLists.txt

find_package(OpenCV CONFIG REQUIRED)

# send_slave: CLI 发送器 (TransmitFile, 默认 4 流; 生产)
add_executable(send_slave
    send_slave.cpp
)

target_link_libraries(send_slave
    PRIVATE
        eyetracker::cfg
        ${OpenCV_LIBS}
)

message(STATUS "send_slave will be built")

# send_ui: 交互式发送器 (角色自动识别, D:/E: 串行 + master 标定附件; 生产)
add_executable(send_ui
    send_ui.cpp
)

target_link_libraries(send_ui
    PRIVATE
        eyetracker::cfg
        ${OpenCV_LIBS}
)

message(STATUS "send_ui will be built")
