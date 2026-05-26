# utils/piper/CMakeLists.txt
add_library(eyetracker_piper STATIC
    src/piper.cpp
)

add_library(eyetracker::piper ALIAS eyetracker_piper)

target_include_directories(eyetracker_piper PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(eyetracker_piper PUBLIC
    eyetracker::cfg
)
