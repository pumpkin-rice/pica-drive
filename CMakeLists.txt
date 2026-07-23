cmake_minimum_required(VERSION 3.22)

project(picadrive)

enable_testing()
enable_language(C ASM)

# 指定库文件输出路径
if (NOT LIBRARY_OUTPUT_DIRECTORY)
    set(LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib)
    set(ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib)
endif()

message(STATUS "LIBRARY_OUTPUT_DIRECTORY: ${LIBRARY_OUTPUT_DIRECTORY}")
message(STATUS "CMake build type: ${CMAKE_BUILD_TYPE}")

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")

    set(CMAKE_VERBOSE_MAKEFILE ON)
endif()

add_subdirectory(src)
add_subdirectory(third)

# add_subdirectory(tests)
