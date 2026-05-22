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

message(STATUS "test_calib_arm will be built")
