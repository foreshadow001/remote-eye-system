# /logger/CmakeLists.txt

add_library(logger INTERFACE)

add_library(eyetracker::logger ALIAS logger)

target_include_directories(logger INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(logger INTERFACE
    eyetracker::core
)

