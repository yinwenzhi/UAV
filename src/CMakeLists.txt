add_library( myslam SHARED
        Frame.cpp
        Feature.cpp
        camera.cpp
        config.cpp
        PointMatch.cpp
        CameraDevice.cpp
        )

target_link_libraries( myslam
        ${THIRD_PARTY_LIBS}
        )