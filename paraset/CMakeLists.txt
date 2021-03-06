cmake_minimum_required(VERSION 2.8.3)
project(paraset)

set(CMAKE_CXX_FLAGS "-g -Wall -std=c++0x")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake/")

set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS 
                    roscpp
                    message_generation
                    argus_utils
                    std_msgs
)

add_message_files( FILES
                   RuntimeParameter.msg
)

# Services
add_service_files( FILES
                   SetRuntimeParameter.srv
                   GetRuntimeParameter.srv
                   GetParameterInfo.srv
)

catkin_python_setup()

generate_messages( DEPENDENCIES
                   std_msgs )

# Use Boost for most utilities, threading
find_package(Boost REQUIRED COMPONENTS random)

catkin_package( INCLUDE_DIRS    include

                LIBRARIES       paraset

                CATKIN_DEPENDS  roscpp 
                                argus_utils
                                message_runtime
                                std_msgs

                DEPENDS         Boost
                                system_lib
)

add_library( paraset
             src/ParasetCommon.cpp
             src/ParamChecks.cpp )
add_dependencies( paraset ${paraset_EXPORTED_TARGETS} )
target_link_libraries( paraset ${catkin_LIBRARIES} )

include_directories( include
                     ${Boost_INCLUDE_DIRS}
                     ${catkin_INCLUDE_DIRS}
)

install( TARGETS paraset
         ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
         LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
         RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark executables and/or libraries for installation
install( DIRECTORY include/${PROJECT_NAME}/
         DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
         FILES_MATCHING PATTERN "*.h *.hpp"
)
