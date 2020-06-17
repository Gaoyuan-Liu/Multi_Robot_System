cmake_minimum_required(VERSION 2.8.3)
project(panda_erg)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
set(CMAKE_CXX_FLAGS "-std=c++14")



find_package(catkin REQUIRED COMPONENTS
  std_msgs
  roscpp
  tf
)

find_package(orocos_kdl)
find_package(kdl_parser)
find_package(Eigen3 REQUIRED)


################################################
## Declare ROS messages, services and actions ##
################################################

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

###################################
## catkin specific configuration ##
###################################

catkin_package()

###########
## Build ##
###########

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)


include_directories(
  SYSTEM ${catkin_INCLUDE_DIRS}
)


add_library(testlib include/testlib/testlib.h
                               src/testlib.cpp)
add_dependencies(testlib        ${catkin_EXPORTED_TARGETS})
target_link_libraries(testlib ${catkin_LIBRARIES} armadillo lapack blas)

add_library(explicit_reference_governor   include/explicit_reference_governor/explicit_reference_governor.h
                                          src/explicit_reference_governor.cpp)
add_dependencies(explicit_reference_governor        ${catkin_EXPORTED_TARGETS})
target_link_libraries(explicit_reference_governor ${catkin_LIBRARIES} armadillo lapack blas)

add_executable(panda_effortcontrol_v1 src/panda_effortcontrol_v1.cpp)
target_link_libraries(panda_effortcontrol_v1 ${catkin_LIBRARIES} )

add_executable(panda_effortcontrol_v2 src/panda_effortcontrol_v2.cpp)
target_link_libraries(panda_effortcontrol_v2 testlib 
                                            ${catkin_LIBRARIES} 
                                            ${kdl_parser_LIBRARIES} 
                                            ${orocos_kdl_LIBRARIES})

add_executable(panda_trajbasedERG_potentialfields src/panda_trajbasedERG_potentialfields.cpp)
target_link_libraries(panda_trajbasedERG_potentialfields  explicit_reference_governor 
                                                          ${catkin_LIBRARIES} 
                                                          ${kdl_parser_LIBRARIES} 
                                                          ${orocos_kdl_LIBRARIES})


#############
## Install ##
#############

# install(DIRECTORY src
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})