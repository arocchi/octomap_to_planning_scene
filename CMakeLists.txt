project("octomap_to_planning_scene")
cmake_minimum_required(VERSION 2.8)

FIND_PACKAGE(Eigen3 REQUIRED)
FIND_PACKAGE(idynutils REQUIRED)
FIND_PACKAGE(iDynTree REQUIRED)
FIND_PACKAGE(moveit_core REQUIRED)
FIND_PACKAGE(rosconsole REQUIRED)
FIND_PACKAGE(rosbag REQUIRED)
FIND_PACKAGE(tf_conversions REQUIRED)
FIND_PACKAGE(tf REQUIRED)

ADD_EXECUTABLE(tf_and_octomap_to_octomap    tf_and_octomap_to_octomap.cpp)

INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIR})
TARGET_LINK_LIBRARIES(tf_and_octomap_to_octomap ${iDynTree_LIBRARIES}
                                                ${idynutils_LIBRARIES}
                                                ${moveit_core_LIBRARIES}
                                                ${rosconsole_LIBRARIES}
                                                ${tf_LIBRARIES})


add_definitions(-DPROJ_DATA_DIR="${CMAKE_CURRENT_BINARY_DIR}/data/")

add_custom_target(copy_data_files ALL
                  ${CMAKE_COMMAND} -E copy_directory "${CMAKE_CURRENT_SOURCE_DIR}/data" "${CMAKE_CURRENT_BINARY_DIR}/data")
