cmake_minimum_required(VERSION 2.8)
project(UAV_PROJECT)

set(CMAKE_CXX_STANDARD 11)
set( CMAKE_BUILD_TYPE "Release" )
#set( CMAKE_BUILD_TYPE "Debug" )
#set( CMAKE_CXX_FLAGS "-std=c++11 -march=native -O3 -pthread"  )
set( CMAKE_CXX_FLAGS "-std=c++11 -O3 -pthread"  )
list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )

set( EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin )
set( LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib )

# set(OpenCV_DIR "/usr/local/opencv343/share/OpenCV")
list( APPEND CMAKE_MODULE_PATH  /home/emast/WorkSpace/install/g2o/cmake_modules )
set( G2O_ROOT /home/emast/WorkSpace/lib/g2o/ )
# include_directories( "/usr/local/include/sophus" )
# set(Sophus_LIBRARIES "/usr/local/lib/libSophus.so")

find_package( G2O REQUIRED )
include_directories( ${G2O_INCLUDE_DIRS} )
find_package(Sophus REQUIRED)
find_package(Pangolin REQUIRED)
find_package(OpenCV  REQUIRED)
find_package(CSparse REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS}
include_directories(
        ${G2O_INCLUDE_DIR})
        ${PROJECT_SOURCE_DIR}/include
        /usr/include/eigen3
        ${Sophus_INCLUDE_DIRS}
        ${CSPARSE_INCLUDE_DIR}
        ${Pangolin_INCLUDE_DIRS}
        )
        
link_directories("/home/emast/WorkSpace/lib/g2o/lib")
set( THIRD_PARTY_LIBS
        ${OpenCV_LIBS}
        ${Sophus_LIBRARIES}
        ${Pangolin_LIBRARIES}
        ${CSPARSE_LIBRARY}
        g2o_core g2o_stuff g2o_types_sba g2o_csparse_extension
        )
add_library(${PROJECT_NAME} SHARED
        src/Config.cpp
        src/Frame.cpp
        src/camera.cpp
        src/Feature.cpp
        src/PointMatch.cpp
        src/CameraDevice.cpp
        src/display.cpp)
target_link_libraries(${PROJECT_NAME} ${THIRD_PARTY_LIBS} -lpthread -lMVSDK  )

add_executable(uav test/demo.cpp)
target_link_libraries(uav ${PROJECT_NAME})
