cmake_minimum_required(VERSION 2.8.3)
project(roadmap_global_planner)
find_package(catkin REQUIRED 
COMPONENTS 
roadmap_global_planner_msgs
 message_generation std_msgs 
 angles 
 costmap_2d 
 dynamic_reconfigure 
 geometry_msgs 
 nav_core 
 navfn 
 nav_msgs 
 pluginlib 
 roscpp 
 tf)


  add_service_files(
  FILES
  GraphRoadmap.srv
)


generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  roadmap_global_planner_msgs
)



catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    costmap_2d
    dynamic_reconfigure
    geometry_msgs
    nav_core
    navfn
    nav_msgs
    pluginlib
    roscpp
    tf
    roadmap_global_planner_msgs
    message_runtime
)

include_directories(
  include
   ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} 
src/dijkstra.cpp
src/orientation_filler.cpp
)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_library(roadmap_global_planner_lib src/planner_core.cpp)
target_link_libraries(roadmap_global_planner_lib ${catkin_LIBRARIES} ${PROJECT_NAME})
add_dependencies(roadmap_global_planner_lib ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})







add_executable(fake
  src/fake.cpp
)
add_dependencies(fake ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(fake
  ${catkin_LIBRARIES}
)

install(FILES roadmap_global_planner_plugin.xml
DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY include/roadmap_global_planner
DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
PATTERN "*~" EXCLUDE
)

install(TARGETS roadmap_global_planner
ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)