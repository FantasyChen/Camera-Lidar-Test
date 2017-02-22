cmake_minimum_required(VERSION 3.6)
project(camera_lidar_test)



FIND_PACKAGE( OpenCV 2.4.11 )
FIND_PACKAGE( PCL 1.7 )
# OpenCV
INCLUDE_DIRECTORIES( ${OpenCV_INCLUDE_DIRS}  )
LINK_LIBRARIES( ${OpenCV_LIBRARY_DIRS} )

# PCL
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )
# CSparse
FIND_PACKAGE( CSparse REQUIRED )
#INCLUDE_DIRECTORIES( ${G2O_INCLUDE_DIR} )
INCLUDE_DIRECTORIES( ${CSPARSE_INCLUDE_DIR} )
# Eigen
FIND_PACKAGE( Eigen3 REQUIRED )
include_directories( ${EIGEN3_INCLUDE_DIR})

### GFLAGS
FIND_PACKAGE(gflags REQUIRED)
INCLUDE_DIRECTORIES(${GFLAGS_INCLUDE_DIRS})
MESSAGE("gflags found ${Gflags_FOUND} at ${GFLAGS_INCLUDE_DIRS} with namespace: ${GFLAGS_NAMESPACE}")



set(CMAKE_CXX_STANDARD 11)

set(SOURCE_FILES main.cpp)
add_executable(camera_lidar_test utils.h utils.cpp  ${SOURCE_FILES})
TARGET_LINK_LIBRARIES(camera_lidar_test  ${OpenCV_LIBS} ${PCL_LIBRARIES} ${CSPARSE_LIBRARY}  ${GFLAGS_LIBRARIES})