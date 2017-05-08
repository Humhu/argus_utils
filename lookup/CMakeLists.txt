cmake_minimum_required(VERSION 2.8.3)
project(lookup)

set(CMAKE_CXX_FLAGS "-g -Wall -std=c++11")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake/")

if (NOT DEFINED CMAKE_BUILD_TYPE OR CMAKE_BUILD_TYPE STREQUAL "")
  message(STATUS "No build type selected, default to Release")
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING "" FORCE)
endif ()
message(STATUS "Build configuration: ${CMAKE_BUILD_TYPE}")

find_package(catkin REQUIRED 
	COMPONENTS		message_generation
					roscpp
					argus_utils
)

# Use Boost for most utilities, threading
find_package(Boost REQUIRED COMPONENTS thread)

catkin_package(
	INCLUDE_DIRS 	include
	
	LIBRARIES 		lookup
					
	CATKIN_DEPENDS 	roscpp 
					argus_utils
					
	DEPENDS 		Boost 
					system_lib
)

catkin_python_setup()

include_directories( include
					 ${Boost_INCLUDE_DIRS}
					 ${catkin_INCLUDE_DIRS}
)

add_library( lookup
	src/LookupInterface.cpp
)
target_link_libraries( lookup
	${Boost_LIBRARIES}
	${catkin_LIBRARIES}
)

add_executable( lookup_registrar nodes/lookup_registrar.cpp )
target_link_libraries( lookup_registrar lookup )

## Mark executables and/or libraries for installation
install(TARGETS lookup
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
	FILES_MATCHING PATTERN "*.h"
)
