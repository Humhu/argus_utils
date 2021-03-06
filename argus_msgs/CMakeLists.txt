cmake_minimum_required(VERSION 2.8.3)
project(argus_msgs)

find_package(catkin REQUIRED COMPONENTS
                    message_generation
                    geometry_msgs
                    nav_msgs
                    std_msgs
                    sensor_msgs
)

## Generate service files
add_message_files( FILES
                  EstimatePerformance.msg
                  FiducialDetection.msg
                  FilterPredictStep.msg
                  FilterStepInfo.msg
                  FilterUpdateStep.msg
                  ImageFiducialDetections.msg
                  MatrixFloat64.msg
                  OdometryArray.msg
                  Point2D.msg
                  TransformWithCovarianceStamped.msg
                  SymmetricFloat64.msg 
)

generate_messages( DEPENDENCIES 
                   std_msgs
                   sensor_msgs
                   nav_msgs
)

catkin_package( CATKIN_DEPENDS message_runtime 
                               std_msgs 
                               sensor_msgs
                               nav_msgs
)
