# tests/utils/piper/CMakeLists.txt

find_package(OpenCV CONFIG REQUIRED)

add_executable(test_calib_arm
    test_calib_arm.cpp
)

target_link_libraries(test_calib_arm
    PRIVATE
        eyetracker::cfg
        ${OpenCV_LIBS}
)

add_executable(test_record_arm_data
    test_record_arm_data.cpp
)

target_link_libraries(test_record_arm_data
    PRIVATE
        eyetracker::cfg
        eyetracker::cam
        ${OpenCV_LIBS}
)

message(STATUS "test_calib_arm will be built")
message(STATUS "test_record_arm_data will be built")
