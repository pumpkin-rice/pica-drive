cmake_minimum_required(VERSION 3.22)

project(picadrive)

enable_language(C ASM)

if (NOT LIBRARY_OUTPUT_DIRECTORY)
    set(LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib)
    set(ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib)
endif()

message(STATUS "LIBRARY_OUTPUT_DIRECTORY: ${LIBRARY_OUTPUT_DIRECTORY}")

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")

    set(CMAKE_VERBOSE_MAKEFILE ON)
endif()

add_subdirectory(src)

if (CMAKE_BUILD_TYPE EQUAL "Debug")
    # add_subdirectory(tests)
endif()
