add_library( rgbdslam SHARED
        frame.cpp
        map.cpp
        config.cpp
        visualization.cpp
        globalOpt.cpp
        visual_odometry.cpp
        )

target_link_libraries(rgbdslam ${THIRD_PARTY_LIBS})