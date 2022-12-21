add_executable(
  bag_process
  road_boundary.cpp
  bag_process.cpp
  road_graph.cpp
  helper.cpp
)

add_executable(
  process_lane
  process_lane.cpp
  )

target_link_libraries(
  bag_process
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
  ${YAML_CPP_LIBS}
  -lbz2
)

target_link_libraries(
  process_lane
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
  ${YAML_CPP_LIBS}
  )

add_executable(
  test_map
  test_map.cpp
  )

target_link_libraries(test_map ${catkin_LIBRARIES})
