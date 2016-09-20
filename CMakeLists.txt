cmake_minimum_required(VERSION 2.8.3)
project(argus_utils)

set(CMAKE_CXX_FLAGS "-g -Wall -std=c++11")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake/")

set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED 
    COMPONENTS      roscpp
                    tf
                    argus_msgs
                    geometry_msgs
)

# Use Boost for utilities
find_package(Boost REQUIRED COMPONENTS random)

# Use Eigen for matrices, linear algebra
find_package(Eigen3 REQUIRED)

# Use Sophus for group algebras
include( ExternalProject )
ExternalProject_Add( sophus_external
  GIT_REPOSITORY git@github.com:strasdat/Sophus.git
  BUILD_COMMAND cmake --build .
  INSTALL_COMMAND make install
  CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${CATKIN_DEVEL_PREFIX}
)
set( sophus_INCLUDE_DIRS ${CATKIN_DEVEL_PREFIX}/include )
message( STATUS ${sophus_INCLUDE_DIRS} )

catkin_package(
    INCLUDE_DIRS    include
                    ${sophus_INCLUDE_DIRS}
                    ${EIGEN3_INCLUDE_DIR}
    
    LIBRARIES       argus_utils
                    
    CATKIN_DEPENDS  roscpp
                    tf
                    argus_msgs
                    geometry_msgs
                    
    DEPENDS         Boost
                    Eigen3
                    system_lib
                    sophus_external
)

catkin_python_setup()

include_directories( include
                     ${Boost_INCLUDE_DIRS}
                     ${EIGEN3_INCLUDE_DIR}
                     ${catkin_INCLUDE_DIRS}
                     ${sophus_INCLUDE_DIRS}
)

add_library( argus_utils
    src/FilterUtils.cpp
    src/GeometryUtils.cpp
    src/ParamUtils.cpp
    src/PoseSE2.cpp
    src/PoseSE3.cpp
    src/MatrixUtils.cpp
    src/Semaphore.cpp
    src/WorkerPool.cpp
    src/YamlUtils.cpp
)
target_link_libraries( argus_utils
    ${Boost_LIBRARIES}
    ${catkin_LIBRARIES}
    yaml-cpp
)
add_dependencies( argus_utils sophus_external ${catkin_EXPORTED_TARGETS} )

add_executable( yaml_test tests/YamlTest.cpp )
target_link_libraries( yaml_test argus_utils ${catkin_LIBRARIES} ${Boost_LIBRARIES} )

add_executable( filter_test tests/FilterTest.cpp )
target_link_libraries( filter_test argus_utils ${catkin_LIBRARIES} ${Boost_LIBRARIES} )

add_executable( matrix_test tests/MatrixTest.cpp )
target_link_libraries( matrix_test argus_utils ${catkin_LIBRARIES} ${Boost_LIBRARIES} )

## Mark executables and/or libraries for installation
install(TARGETS argus_utils yaml_test matrix_test
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

file(GLOB argus_utils_HEADERS
    "include/${PROJECT_NAME}/*.h"
    "include/${PROJECT_NAME}/*.hpp"
)
install(FILES ${argus_utils_HEADERS}
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)