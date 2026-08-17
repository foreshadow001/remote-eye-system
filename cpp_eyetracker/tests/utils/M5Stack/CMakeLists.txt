# tests/utils/M5Stack/CMakeLists.txt

find_package(OpenCV CONFIG REQUIRED)

add_executable(test_m5stack
    test_m5stack.cpp
)

target_link_libraries(test_m5stack
    PRIVATE
        eyetracker::cfg
        ${OpenCV_LIBS}
)

message(STATUS "test_m5stack will be built")
