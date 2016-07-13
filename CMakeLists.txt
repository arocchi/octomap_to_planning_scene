project("octomap_to_planning_scene")
cmake_minimum_required(VERSION 2.8)

FIND_PACKAGE(Eigen3 REQUIRED)
FIND_PACKAGE(idynutils REQUIRED)
FIND_PACKAGE(iDynTree REQUIRED)
FIND_PACKAGE(moveit_core REQUIRED)
FIND_PACKAGE(rosconsole REQUIRED)
FIND_PACKAGE(rosbag REQUIRED)


ADD_EXECUTABLE(octomap_to_planning_scene octomap_to_planning_scene.cpp)

INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIR})
TARGET_LINK_LIBRARIES(octomap_to_planning_scene ${iDynTree_LIBRARIES}
                                                ${idynutils_LIBRARIES}
                                                ${moveit_core_LIBRARIES}
                                                ${rosconsole_LIBRARIES}
                                                ${rosbag_LIBRARIES})

add_definitions(-DPROJ_DATA_DIR="${CMAKE_CURRENT_BINARY_DIR}/data/")

add_custom_target(copy_data_files ALL
                  ${CMAKE_COMMAND} -E copy_directory "${CMAKE_CURRENT_SOURCE_DIR}/data" "${CMAKE_CURRENT_BINARY_DIR}/data")
