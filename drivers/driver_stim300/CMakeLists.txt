cmake_minimum_required(VERSION 3.8)
project(driver_stim300)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_library(driver_stim300
  src/driver_stim300.cpp
  src/serial_unix.cpp
  src/datagram_parser.cpp
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(sam_msgs REQUIRED)


# Executable for the node
add_executable(stim300_driver src/stim300_driver_node.cpp)
target_link_libraries(stim300_driver driver_stim300)
ament_target_dependencies(stim300_driver
  rclcpp 
  sensor_msgs
  sam_msgs
)

install(TARGETS
  driver_stim300
  stim300_driver
  DESTINATION lib/${PROJECT_NAME}
)
if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  ament_add_gmock(check_driver_stim300
    test/check_driver_stim300.cpp
    test/mock_serial_driver.h
  )
  if(TARGET check_driver_stim300)
    target_link_libraries(check_driver_stim300 driver_stim300)
  endif()
endif()

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(check_datasheet_constanst
    test/check_stim300_constants.cpp
  ) 
  if(TARGET check_datasheet_constanst)
    target_link_libraries(check_datasheet_constanst driver_stim300)
  endif()
endif()
# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   ament_lint_auto_find_test_dependencies()

#   # Add test for the driver
#   ament_add_gtest(check_driver_stim300
#     test/check_driver_stim300.cpp
#     test/mock_serial_driver.h
#   )
#     target_link_libraries(check_driver_stim300 driver_stim300)
#   endif()

#   # Add test for the constants
#   ament_add_gtest(check_datasheet_constanst
#     test/check_stim300_constants.cpp
#   )
#   if(TARGET check_datasheet_constanst)
#     target_link_libraries(check_datasheet_constanst
#       driver_stim300
#       GTest::GTest
#       GTest::Main
#       gmock
#     )
#   endif()

  # add_test(NAME check_driver_stim300 COMMAND check_driver_stim300)
  # add_test(NAME check_datasheet_constanst COMMAND check_datasheet_constanst)
# endif()

ament_package()
