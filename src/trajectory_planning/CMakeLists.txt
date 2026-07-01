cmake_minimum_required(VERSION 3.18)

include(GoogleTest)

add_library(trajectory_planning STATIC)
set(SRC
    velocity_trapezoidal.cpp
)

set_target_properties(trajectory_planning
    PROPERTIES
    ARCHIVE_OUTPUT_DIRECTORY "${ARCHIVE_OUTPUT_DIRECTORY}"
    LIBRARY_OUTPUT_DIRECTORY "${ARCHIVE_OUTPUT_DIRECTORY}"
    RUNTIME_OUTPUT_DIRECTORY "${ARCHIVE_OUTPUT_DIRECTORY}"
)

target_include_directories(trajectory_planning
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}
        .. # for utils/math.h
)

message(STATUS "")

target_sources(trajectory_planning
    PRIVATE
        ${SRC}
)

target_link_libraries(trajectory_planning
    PUBLIC
        
)

enable_testing()
find_package(GTest REQUIRED)
include(GoogleTest)

add_executable(trajectory_planning_test
    velocity_trapezoidal_test.cpp
)

target_link_libraries(trajectory_planning_test
    trajectory_planning
    matplotlib-cpp
    GTest::gtest_main
)
gtest_discover_tests(trajectory_planning_test)
