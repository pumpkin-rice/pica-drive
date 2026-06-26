# FreeRTOS internal cmake file. Do not use it in user top-level project

add_library(matplotlib-cpp INTERFACE)

# 查找依赖
find_package(Python3 COMPONENTS Interpreter Development)
find_package(Python3 COMPONENTS NumPy REQUIRED)

target_include_directories(matplotlib-cpp
    INTERFACE
        .
)

target_link_libraries(matplotlib-cpp
    INTERFACE
        Python3::Python
        Python3::Module
        Python3::NumPy
)

find_package(GTest REQUIRED)
enable_testing()
include(GoogleTest)

add_executable(matplotlib_test
    matplotlib_test.cpp
)

target_link_libraries(matplotlib_test
    matplotlib-cpp
    GTest::gtest_main
)
gtest_discover_tests(matplotlib_test)
