# PicaDrive internal cmake file. Do not use it in user top-level project

add_subdirectory(trajectory_planning)

add_library(picadrive STATIC)

if(PICADRIVE_ENABLE_LOGGER)
    add_definitions(-DPICA_DRIVE_ENABLE_LOGGER=1)
endif()

set(SRC
    bldc/bldc.cc
    bldc/svpwm.c
    bldc/svpwm.c
    bldc/current_controller/current_controller_foc.cc
    bldc/current_controller/phase_inductance_measurer.cc
    bldc/current_controller/phase_resistance_measurer.cc
    bldc/speed_controller/speed_controller_pi.cc
)

set_target_properties(picadrive
    PROPERTIES
    ARCHIVE_OUTPUT_DIRECTORY "${ARCHIVE_OUTPUT_DIRECTORY}"
    LIBRARY_OUTPUT_DIRECTORY "${ARCHIVE_OUTPUT_DIRECTORY}"
    RUNTIME_OUTPUT_DIRECTORY "${ARCHIVE_OUTPUT_DIRECTORY}"
)

target_include_directories(picadrive
    PUBLIC
        .
        ${PROJECT_SOURCE_DIR}/third/matrix
        ${PROJECT_SOURCE_DIR}/ports

    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}
)

target_sources(picadrive
    PRIVATE
        ${SRC}
)

if(PICADRIVE_ENABLE_LOGGER)
    target_link_libraries(picadrive
        PUBLIC
            spdlog::spdlog_header_only
    )

else()
    target_link_libraries(picadrive
        PUBLIC
    )

endif()


