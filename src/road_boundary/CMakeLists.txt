add_executable(
  ${PROJECT_NAME}
  road_boundary.cpp
  node.cpp
  main.cpp
)

target_link_libraries(
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  -lbz2
)

add_executable(
  bag_process
  road_boundary.cpp
  bag_process.cpp
)

target_link_libraries(
  bag_process
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  -lbz2
)

add_executable(
  test_map
  test_map.cpp
  )

target_link_libraries(test_map ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
