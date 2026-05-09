# utils/cam_calib/CMakeLists.txt

add_executable(cam_calib
    calib_cam.cpp
)

if(NOT DEFINED ENV{HALCONROOT})
    message(WARNING "⚠️  HALCONROOT not set. Please specify path to HALCON SDK, e.g.:")
    message(WARNING "    cmake -D HALCONROOT=\"C:/Program Files/MVTec/HALCON-21.11-Progress\" ..")
endif()

set(HALCONROOT $ENV{HALCONROOT})

target_include_directories(cam_calib PUBLIC
    "${HALCONROOT}/include"
    "${HALCONROOT}/include/halconcpp"
)

target_link_directories(cam_calib PUBLIC
    "${HALCONROOT}/lib/x64-win64"
)

target_link_libraries(cam_calib PUBLIC
    halcon.lib
    halconcpp.lib
    eyetracker::cfg
)

message(STATUS "✅ cam_calib will be built")

add_executable(cam_calib_chain
    calib_cam_chain.cpp
)

target_include_directories(cam_calib_chain PUBLIC
    "${HALCONROOT}/include"
    "${HALCONROOT}/include/halconcpp"
)

target_link_directories(cam_calib_chain PUBLIC
    "${HALCONROOT}/lib/x64-win64"
)

target_link_libraries(cam_calib_chain PUBLIC
    halcon.lib
    halconcpp.lib
    eyetracker::cfg
)

message(STATUS "✅ cam_calib_chain will be built")
