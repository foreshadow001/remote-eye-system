# tests/utils/cam_calib/CMakeLists.txt

# ---- test_calib_images: 不需要 HALCON ----
find_package(OpenCV CONFIG REQUIRED)

add_executable(test_calib_images
    test_calib_images.cpp
)

target_link_libraries(test_calib_images
    PRIVATE
        eyetracker::cfg
        eyetracker::cam
        ${OpenCV_LIBS}
)

message(STATUS "✅ test_calib_images will be built")

# ---- cam_calib / cam_calib_chain: 需要 HALCON ----
if(NOT DEFINED ENV{HALCONROOT})
    message(WARNING "⚠️  HALCONROOT not set. Skipping cam_calib and cam_calib_chain.")
    message(WARNING "    cmake -D HALCONROOT=\"C:/Program Files/MVTec/HALCON-21.11-Progress\" ..")
else()
    set(HALCONROOT $ENV{HALCONROOT})

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
endif()
