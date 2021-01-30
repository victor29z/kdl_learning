cmake_minimum_required(VERSION 2.8.3)
project(kdl_test)

## Compile as C++11, supported in ROS Kinetic and newer
#add_compile_options(-std=c++11)

set(CMAKE_CXX_FLAGS "-std=c++11 -Wignored-attributes ${CMAKE_CXX_FLAGS}")
add_definitions(-w)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(
  catkin  REQUIRED  COMPONENTS
  roscpp
  rospy
  std_msgs
  kdl_parser
  urdf
  eigen_conversions
)


# find packages
find_package(Eigen3 REQUIRED)
find_package(orocos_kdl REQUIRED)


## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES kdl_test
   CATKIN_DEPENDS roscpp std_msgs 
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
${catkin_INCLUDE_DIRS}
${orocos_kdl_INCLUDE_DIRS}
"/usr/include/eigen3"
)

link_directories(${catkin_LIBRARY_DIRS})


add_executable(fkine src/fkine.cpp)
add_executable(ur_iksolver src/ur_iksolver.cpp)
add_executable(xmate_iksolver src/xmate_iksolver.cpp)
add_executable(master_control_xmate src/master_control_xmate.cpp)
add_executable(master_control_xmate_clik src/master_control_xmate_clik.cpp)
add_executable(ae_predict_constraint_clik src/ae_predict_constraint_clik.cpp)
add_executable(ae_online_training src/ae_online_training.cpp)
#add_executable(master_control_lbr_clik src/master_control_lbr_clik.cpp)
#add_executable(master_control_yumi src/master_control_yumi.cpp)
#add_executable(testnode src/testnode.cpp)
#add_executable(fk_data_conv src/fk_data_conv.cpp)


## Specify libraries to link a library or executable target against
#target_link_libraries(testlib orocos-kdl)
target_link_libraries(fkine orocos-kdl)
target_link_libraries(fkine kdl_parser)

target_link_libraries(ur_iksolver orocos-kdl)
target_link_libraries(ur_iksolver kdl_parser)
target_link_libraries(ur_iksolver ${catkin_LIBRARIES})

target_link_libraries(xmate_iksolver orocos-kdl)
target_link_libraries(xmate_iksolver kdl_parser)
target_link_libraries(xmate_iksolver ${catkin_LIBRARIES})

target_link_libraries(master_control_xmate orocos-kdl)
target_link_libraries(master_control_xmate kdl_parser)
target_link_libraries(master_control_xmate ${catkin_LIBRARIES})

target_link_libraries(master_control_xmate_clik orocos-kdl)
target_link_libraries(master_control_xmate_clik kdl_parser)
target_link_libraries(master_control_xmate_clik eigen_conversions)
target_link_libraries(master_control_xmate_clik ${catkin_LIBRARIES})

target_link_libraries(ae_predict_constraint_clik orocos-kdl kdl_parser eigen_conversions ${catkin_LIBRARIES})

target_link_libraries(ae_online_training orocos-kdl kdl_parser eigen_conversions ${catkin_LIBRARIES})

#target_link_libraries(master_control_lbr_clik orocos-kdl)
#target_link_libraries(master_control_lbr_clik kdl_parser)
#target_link_libraries(master_control_lbr_clik eigen_conversions)
#target_link_libraries(master_control_lbr_clik ${catkin_LIBRARIES})

#target_link_libraries(master_control_yumi orocos-kdl)
#target_link_libraries(master_control_yumi kdl_parser)
#target_link_libraries(master_control_yumi eigen_conversions)
#target_link_libraries(master_control_yumi ${catkin_LIBRARIES})

#target_link_libraries(fk_data_conv orocos-kdl)
#target_link_libraries(fk_data_conv kdl_parser)
#target_link_libraries(fk_data_conv eigen_conversions)
#target_link_libraries(fk_data_conv ${catkin_LIBRARIES})

#target_link_libraries(testnode orocos-kdl)
#target_link_libraries(testnode kdl_parser)
#target_link_libraries(testnode ${catkin_LIBRARIES})


