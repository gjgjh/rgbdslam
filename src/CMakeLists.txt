add_library( rgbdslam SHARED
        common_include.cpp
        frame.cpp
        map.cpp
        config.cpp
        visualization.cpp
        looper.cpp
        globalOpt.cpp
        visual_odometry.cpp
        )

target_link_libraries(rgbdslam ${THIRD_PARTY_LIBS})