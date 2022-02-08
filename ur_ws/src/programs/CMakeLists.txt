cmake_minimum_required(VERSION 3.0.2)

project(programs)

find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  roscpp
  sensor_msgs
)
catkin_package(
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

#add_executable(pcl_filtered src/getwork.cpp)
add_executable(table_pcd src/table_pcd.cpp)
add_executable(tabler_pcd src/tabler_pcd.cpp)
add_executable(pcd_viewer src/pcd_viewer.cpp)
#target_link_libraries(pcl_filtered
 #  ${catkin_LIBRARIES}
 #)
target_link_libraries(table_pcd
   ${catkin_LIBRARIES}
)
target_link_libraries(tabler_pcd
   ${catkin_LIBRARIES}
 )


target_link_libraries(pcd_viewer
   ${catkin_LIBRARIES}
 )
