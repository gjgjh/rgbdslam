add_executable(run_vo run_vo.cpp)
target_link_libraries(run_vo ${THIRD_PARTY_LIBS} rgbdslam)

add_executable(map_saver map_saver.cpp)
target_link_libraries(map_saver ${THIRD_PARTY_LIBS} rgbdslam)