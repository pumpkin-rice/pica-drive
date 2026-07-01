# PicaDrive internal cmake file. Do not use it in user top-level project

add_subdirectory(trajectory_planning)

add_library(picadrive STATIC)

set(SRC
    motor/config.c
    motor/pwm/svm.c
    motor/bldc/foc.cc
    motor/bldc/bldc.cc
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
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}
)

message(STATUS "")

target_sources(picadrive
    PRIVATE
        ${SRC}
)

target_link_libraries(picadrive
    PUBLIC
)
