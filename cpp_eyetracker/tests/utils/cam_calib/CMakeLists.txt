# tests/utils/cam_calib/CMakeLists.txt

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

message(STATUS "test_calib_images will be built")

add_executable(calib_with_HALCON
    calib_with_HALCON.cpp
)

target_link_libraries(calib_with_HALCON
    PRIVATE
        eyetracker::cfg
        eyetracker::cam
        ${OpenCV_LIBS}
)

# [DBG] 帧级诊断版本 (坏帧落盘 + 缓冲指针跟踪)
add_executable(calib_debug
    calib_debug.cpp
)

target_link_libraries(calib_debug
    PRIVATE
        eyetracker::cfg
        eyetracker::cam
        ${OpenCV_LIBS}
)

# [Verify] Pylon Viewer 式取流 (GrabLoop_ProvidedByUser + RetrieveResult)
add_executable(calib_retrieve
    calib_retrieve.cpp
)

target_link_libraries(calib_retrieve
    PRIVATE
        eyetracker::cfg
        eyetracker::cam
        ${OpenCV_LIBS}
)

message(STATUS "✅ calib_with_HALCON will be built")

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

    # Copy HALCON DLLs to output directory for runtime
    add_custom_command(TARGET cam_calib_chain POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${HALCONROOT}/bin/x64-win64/halcon.dll"
            "${HALCONROOT}/bin/x64-win64/halconcpp.dll"
            $<TARGET_FILE_DIR:cam_calib_chain>
        COMMENT "Copying HALCON DLLs to output..."
    )

    message(STATUS "✅ cam_calib_chain will be built")
endif()
