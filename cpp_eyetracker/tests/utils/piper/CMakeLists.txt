# tests/utils/piper/CMakeLists.txt

find_package(OpenCV CONFIG REQUIRED)

# ---- test_calib_arm / test_record_arm_data: 不需要 HALCON ----

add_executable(get_piper_pose
    get_piper_pose.cpp
)

target_link_libraries(get_piper_pose
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

add_executable(test_piper_to_cam
    test_piper_to_cam.cpp
)

target_link_libraries(test_piper_to_cam
    PRIVATE
        eyetracker::piper
        eyetracker::cfg
        ${OpenCV_LIBS}
)

message(STATUS "get_piper_pose will be built")
message(STATUS "test_record_arm_data will be built")
message(STATUS "test_piper_to_cam will be built")

# ---- resolve_calib_board_pose: 需要 HALCON ----
if(NOT DEFINED ENV{HALCONROOT})
    message(WARNING "HALCONROOT not set. Skipping resolve_calib_board_pose.")
else()
    set(HALCONROOT $ENV{HALCONROOT})

    add_executable(resolve_calib_board_pose
        resolve_calib_board_pose.cpp
    )

    target_include_directories(resolve_calib_board_pose PUBLIC
        "${HALCONROOT}/include"
        "${HALCONROOT}/include/halconcpp"
    )

    target_link_directories(resolve_calib_board_pose PUBLIC
        "${HALCONROOT}/lib/x64-win64"
    )

    target_link_libraries(resolve_calib_board_pose PUBLIC
        halcon.lib
        halconcpp.lib
        eyetracker::cfg
    )

    add_custom_command(TARGET resolve_calib_board_pose POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${HALCONROOT}/bin/x64-win64/halcon.dll"
            "${HALCONROOT}/bin/x64-win64/halconcpp.dll"
            $<TARGET_FILE_DIR:resolve_calib_board_pose>
        COMMENT "Copying HALCON DLLs..."
    )

    message(STATUS "resolve_calib_board_pose will be built")
endif()
