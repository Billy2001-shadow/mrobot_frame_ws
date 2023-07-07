cmake_minimum_required(VERSION 2.8.3)
project(mrobot_frame)

SET(CMAKE_BUILD_TYPE "Release")
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")
add_compile_options(-std=c++14)
add_definitions(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  laser_geometry
  pcl_conversions
  pcl_ros
  roscpp
  rospy
  geometry_msgs
  eigen_conversions
  message_generation 
  sensor_msgs
  tf
  std_srvs
  std_msgs
)

add_service_files(
   FILES
   saveMap.srv
   optimizeMap.srv
   saveGridmap.srv
)

generate_messages(
   DEPENDENCIES
   std_msgs
)

set(ALL_TARGET_LIBRARIES "")

include(cmake/glog.cmake)
include(cmake/YAML.cmake)
include(cmake/PCL.cmake)
include(cmake/eigen.cmake)
include(cmake/geographic.cmake)
include(cmake/g2o.cmake)

include_directories(include ${catkin_INCLUDE_DIRS})
include(cmake/global_defination.cmake)
catkin_package()

file(GLOB_RECURSE ALL_SRCS "*.cpp")
file(GLOB_RECURSE NODE_SRCS "src/apps/*_node.cpp")
file(GLOB_RECURSE THIRD_PARTY_SRCS "third_party/*.cpp")
list(REMOVE_ITEM ALL_SRCS ${NODE_SRCS})
list(REMOVE_ITEM ALL_SRCS ${THIRD_PARTY_SRCS})


add_executable(${PROJECT_NAME}_data_pretreat_node src/apps/data_pretreat_node.cpp ${ALL_SRCS})
add_dependencies(${PROJECT_NAME}_data_pretreat_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_data_pretreat_node ${catkin_LIBRARIES} ${ALL_TARGET_LIBRARIES})

add_executable(${PROJECT_NAME}_front_end_node src/apps/front_end_node.cpp ${ALL_SRCS})
add_dependencies(${PROJECT_NAME}_front_end_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_front_end_node ${catkin_LIBRARIES} ${ALL_TARGET_LIBRARIES})

add_executable(${PROJECT_NAME}_back_end_node src/apps/back_end_node.cpp ${ALL_SRCS})
add_dependencies(${PROJECT_NAME}_back_end_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_back_end_node ${catkin_LIBRARIES} ${ALL_TARGET_LIBRARIES})

add_executable(${PROJECT_NAME}_loop_closing_node src/apps/loop_closing_node.cpp ${ALL_SRCS})
add_dependencies(${PROJECT_NAME}_loop_closing_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_loop_closing_node ${catkin_LIBRARIES} ${ALL_TARGET_LIBRARIES})

add_executable(${PROJECT_NAME}_viewer_node src/apps/viewer_node.cpp ${ALL_SRCS})
add_dependencies(${PROJECT_NAME}_viewer_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_viewer_node ${catkin_LIBRARIES} ${ALL_TARGET_LIBRARIES})

add_executable(${PROJECT_NAME}_mapping_node src/apps/mapping_node.cpp ${ALL_SRCS})
add_dependencies(${PROJECT_NAME}_mapping_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_mapping_node ${catkin_LIBRARIES} ${ALL_TARGET_LIBRARIES})