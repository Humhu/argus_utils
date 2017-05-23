cmake_minimum_required(VERSION 2.8.3)
project(vizard)

set(CMAKE_CXX_FLAGS "-g -Wall -std=c++0x")

if( NOT DEFINED CMAKE_BUILD_TYPE OR CMAKE_BUILD_TYPE STREQUAL "" )
    message( STATUS "No build type selected, default to Release" )
    set( CMAKE_BUILD_TYPE "Release" CACHE STRING "" FORCE )
endif()
message( STATUS "Build configuration: ${CMAKE_BUILD_TYPE}" )

find_package( catkin REQUIRED 
              COMPONENTS roscpp
                         std_msgs
                         geometry_msgs
                         visualization_msgs
                         argus_utils
                         argus_msgs
                         extrinsics_array
)

# Use Boost for most utilities, threading
find_package(Boost REQUIRED)

catkin_package( INCLUDE_DIRS    include

                LIBRARIES       vizard

                CATKIN_DEPENDS  roscpp 
                                std_msgs
                                geometry_msgs
                                visualization_msgs                                
                                argus_utils
                                argus_msgs
                                extrinsics_array

               DEPENDS          Boost 
                                system_lib
)

include_directories( include 
                    ${Boost_INCLUDE_DIRS}
                    ${catkin_INCLUDE_DIRS}
)

add_library( vizard
             src/Visualizer.cpp
             src/PoseVisualizer.cpp
)
add_dependencies( vizard ${vizard_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} )
target_link_libraries( vizard
                       ${Boost_LIBRARIES}
                       ${catkin_LIBRARIES}
)

# add_executable( fiducial_array_calibrator_node nodes/fiducial_array_calibrator.cpp )
# add_dependencies( fiducial_array_calibrator_node ${PROJECT_NAME}_gencfg )
# target_link_libraries( fiducial_array_calibrator_node 
#                        ${isam_LIBRARIES}
#                        ${catkin_LIBRARIES}
#                        vizard
# )

## Mark executables and/or libraries for installation
install(TARGETS vizard
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

file(GLOB vizard_HEADERS
     "include/${PROJECT_NAME}/*.h"
     "include/${PROJECT_NAME}/*.hpp"
)
install(FILES ${vizard_HEADERS}
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
