cmake_minimum_required(VERSION 3.5)
project(cloudwatch_metrics_collector)

## Compile as C++11, supported in ROS Kinetic and newer
set(CMAKE_CXX_STANDARD 14)

# Enable strict compiler flags if possible
include(CheckCXXCompilerFlag)
set(FLAGS -Wno-long-long -Wall -Wextra -Wcast-align -Wcast-qual -Wformat -Wwrite-strings)
foreach(FLAG ${FLAGS})
  check_cxx_compiler_flag(${FLAG} R${FLAG})
  if(${R${FLAG}})
    set(WARNING_CXX_FLAGS "${WARNING_CXX_FLAGS} ${FLAG}")
  endif()
endforeach()

if(NOT DEFINED CXX_DISABLE_WERROR)
  set(WARNING_CXX_FLAGS "-Werror ${WARNING_CXX_FLAGS}")
endif()
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${WARNING_CXX_FLAGS}")

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

find_package(ament_cmake REQUIRED)
find_package(aws_common REQUIRED)
find_package(aws_ros2_common REQUIRED)
find_package(cloudwatch_metrics_common REQUIRED)
find_package(dataflow_lite REQUIRED)
find_package(file_management REQUIRED)
find_package(rmw_implementation REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(ros_monitoring_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)

###########
## Build ##
###########

## Specify additional locations of header files
set(cloudwatch_metrics_ros2_INCS
  include
  ${ament_cmake_INCLUDE_DIRS}
  ${aws_common_INCLUDE_DIRS}
  ${aws_ros2_common_INCLUDE_DIRS}
  ${cloudwatch_metrics_common_INCLUDE_DIRS}
  ${dataflow_lite_INCLUDE_DIRS}
  ${file_management_INCLUDE_DIRS}
  ${rmw_implementation_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  ${rosgraph_msgs_INCLUDE_DIRS}
  ${ros_monitoring_msgs_INCLUDE_DIRS}
  ${std_srvs_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
)

set(cloudwatch_metrics_ros2_LIBS
  ${ament_cmake_LIBRARIES}
  ${aws_common_LIBRARIES}
  ${aws_ros2_common_LIBRARIES}
  ${cloudwatch_metrics_common_LIBRARIES}
  ${cloudwatch_metrics_collector_LIBRARIES}
  ${dataflow_lite_LIBRARIES}
  ${file_management_LIBRARIES}
  ${rmw_implementation_LIBRARIES}
  ${rclcpp_LIBRARIES}
  ${rosgraph_msgs_LIBRARIES}
  ${ros_monitoring_msgs_LIBRARIES}
  ${std_srvs_LIBRARIES}
  ${std_msgs_LIBRARIES}
)

## Declare a C++ executable
add_library(${PROJECT_NAME}_lib SHARED src/metrics_collector.cpp src/metrics_collector_parameter_helper.cpp)

target_include_directories(${PROJECT_NAME}_lib PUBLIC ${cloudwatch_metrics_ros2_INCS})
target_link_libraries(${PROJECT_NAME}_lib ${cloudwatch_metrics_ros2_LIBS})

add_executable(${PROJECT_NAME} src/main.cpp)
target_include_directories(${PROJECT_NAME} PRIVATE ${cloudwatch_metrics_ros2_INCS})
target_link_libraries(${PROJECT_NAME} ${PROJECT_NAME}_lib ${cloudwatch_metrics_ros2_LIBS})

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS ${PROJECT_NAME}_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib
)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(aws_common)
ament_export_dependencies(aws_ros2_common)
ament_export_dependencies(cloudwatch_metrics_common)
ament_export_dependencies(dataflow_lite)
ament_export_dependencies(file_management)
ament_export_dependencies(rmw_implementation)
ament_export_dependencies(rclcpp)
ament_export_dependencies(rosgraph_msgs)
ament_export_dependencies(ros_monitoring_msgs)
ament_export_dependencies(std_srvs)
ament_export_dependencies(std_msgs)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_lib ${cloudwatch_metrics_ros2_LIBS})
ament_package()

#############
## Tests ##
#############

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  
  ament_add_gmock(test_cloudwatch_metrics_collector
          test/cloudwatch_metrics_collector_test.cpp
          )
  target_include_directories(test_cloudwatch_metrics_collector PRIVATE
          ${cloudwatch_metrics_ros2_INCS}
          )
  target_link_libraries(test_cloudwatch_metrics_collector
          ${PROJECT_NAME}_lib
          )
endif()
