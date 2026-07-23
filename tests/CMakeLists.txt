cmake_minimum_required(VERSION 3.22)

if(CMAKE_C_STANDARD LESS 11)
	message(ERROR "Generated code requires C11 or higher")
endif()

enable_testing()
find_package(GTest REQUIRED)
include(GoogleTest)

# add_executable(test_bldc_foc
# 	test_bldc_current_foc.cc
# )
# target_link_directories(test_bldc_foc
# 	PRIVATE
# 		${CMAKE_ARCHIVE_OUTPUT_DIRECTORY}
# 		${CMAKE_LIBRARY_OUTPUT_DIRECTORY}
# )
# target_link_libraries(test_bldc_foc
# 	PRIVATE
# 		picadrive
# 		GTest::gtest_main
# 		matplotlib-cpp
# )
# gtest_discover_tests(test_bldc_foc)

add_executable(test_bldc_speed_pi
	test_bldc_speed_pi.cc
)
target_link_directories(test_bldc_speed_pi
	PRIVATE
		${CMAKE_ARCHIVE_OUTPUT_DIRECTORY}
		${CMAKE_LIBRARY_OUTPUT_DIRECTORY}
)
target_link_libraries(test_bldc_speed_pi
	PRIVATE
		picadrive
		GTest::gtest_main
		matplotlib-cpp
)
gtest_discover_tests(test_bldc_speed_pi)
