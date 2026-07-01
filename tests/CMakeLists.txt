cmake_minimum_required(VERSION 3.22)

if(CMAKE_C_STANDARD LESS 11)
	message(ERROR "Generated code requires C11 or higher")
endif()

enable_testing()
find_package(GTest REQUIRED)
include(GoogleTest)

# add_executable(test_bldc_foc EXCLUDE_FROM_ALL)

add_executable(test_bldc_foc
	test_bldc_foc.cc
)
set_target_properties(test_bldc_foc
	PROPERTIES
		RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin"
)

target_include_directories(test_bldc_foc
	PRIVATE
)

target_link_directories(test_bldc_foc
	PRIVATE
		${CMAKE_ARCHIVE_OUTPUT_DIRECTORY}
		${CMAKE_LIBRARY_OUTPUT_DIRECTORY}
)

target_link_libraries(test_bldc_foc
	PRIVATE
		picadrive
		GTest::gtest_main
		matplotlib-cpp
)
gtest_discover_tests(test_bldc_foc)
