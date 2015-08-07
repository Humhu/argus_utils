cmake_minimum_required(VERSION 2.8.3)
project(argus_utils)

set(CMAKE_CXX_FLAGS "-g -Wall -std=c++11")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake/")

set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED 
	COMPONENTS		roscpp
					tf
					geometry_msgs
)

# Use Boost for utilities
find_package(Boost REQUIRED)

# Use Eigen for matrices, linear algebra
find_package(Eigen3 REQUIRED)

catkin_package(
	INCLUDE_DIRS 	include
	
	LIBRARIES 		argus_utils
					
	CATKIN_DEPENDS 	roscpp
					
	DEPENDS 		Boost
					Eigen3
					system_lib
					tf
					geometry_msgs
)

include_directories( include
					 ${Boost_INCLUDE_DIRS}
					 ${EIGEN3_INCLUDE_DIR}
					 ${catkin_INCLUDE_DIRS}
)

add_library( argus_utils
	src/GeometryUtils.cpp
	src/PoseSE2.cpp
	src/PoseSE3.cpp
	src/YamlUtils.cpp
)
# add_dependencies( argus_utils ${argus_utils_EXPORTED_TARGETS} )
target_link_libraries( argus_utils
	${Boost_LIBRARIES}
	${catkin_LIBRARIES}
	yaml-cpp
)

add_executable( yaml_test tests/YamlTest.cpp )
target_link_libraries( yaml_test argus_utils )

## Mark executables and/or libraries for installation
install(TARGETS argus_utils 
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
