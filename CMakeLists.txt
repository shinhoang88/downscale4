cmake_minimum_required(VERSION 2.8.3)
project(downscale4)
add_compile_options(-std=c++11)
find_package(catkin REQUIRED COMPONENTS
   roscpp 
  gazebo_ros 
  gazebo
  std_msgs
  message_generation
)
#SET(GCC_COVERAGE_COMPILE_FLAGS "-D_REENTRANT -fpic -O2 -Wall")
#ADD_COMPILE_OPTIONS( -D_REENTRANT -fpic -Wall) #ADD

#SET(GCC_COVERAGE_LINK_FLAGS    "-lPowerPMACcontrol -lssh2 -lrt")

SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} ${GCC_COVERAGE_COMPILE_FLAGS}")
SET(CMAKE_EXE_LINKER_FLAGS  "${CMAKE_EXE_LINKER_FLAGS} ${GCC_COVERAGE_LINK_FLAGS}")
## Declare ROS messages and services
add_message_files(FILES one.msg two.msg three.msg seven.msg trans.msg buttons.msg charmsg.msg eetorque.msg)


## Generate services in the 'srv' folder
#add_service_files(
#  FILES
#  MoveArm.srv
#)
## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

#catkin_package(
#    DEPENDS 
#    CATKIN_DEPENDS roscpp std_msgs message_runtime 
#    INCLUDE_DIRS 
#    LIBRARIES 

# Depend on system install of Gazebo
find_package(gazebo REQUIRED)

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

include_directories(
	include
)

 #ADD DIRECTORIES FOR LIBRARY usr/bin/ld error !
LINK_DIRECTORIES ( 
	lib	
	/home/robotory/catkin_ws/devel/lib
	
)

#addition package

find_package(orocos_kdl)
find_package(kdl_parser)

###sub
add_library(sub_downscale src/sub_downscale.cpp)
target_link_libraries(sub_downscale ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(sub_downscale downscale4_generate_messages_cpp)

#add_library(sub_downscale_dual_arm src/sub_downscale_dual_arm.cpp)
#target_link_libraries(sub_downscale_dual_arm ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})
#add_dependencies(sub_downscale_dual_arm downscale4_generate_messages_cpp)

#add_library(sub_dualdownscale src/sub_dualdownscale.cpp)
#target_link_libraries(sub_dualdownscale ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})
#add_dependencies(sub_dualdownscale downscale4_generate_messages_cpp)

add_executable(torqueopt src/torqueopt.cpp)
target_link_libraries(torqueopt ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(torqueopt downscale4_generate_messages_cpp)

add_executable(torqueopt_testcomputingtime src/torqueopt_testcomputingtime.cpp)
target_link_libraries(torqueopt_testcomputingtime ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(torqueopt_testcomputingtime downscale4_generate_messages_cpp)

#add_executable(torqueopt_testcomputingtimev2 src/torqueopt_testcomputingtimev2.cpp)
#target_link_libraries(torqueopt_testcomputingtimev2 ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})
#add_dependencies(torqueopt_testcomputingtimev2 downscale4_generate_messages_cpp)

#add_executable(torqueopt_testcomputingtimev3 src/torqueopt_testcomputingtimev3.cpp)
#target_link_libraries(torqueopt_testcomputingtimev3 ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})
#add_dependencies(torqueopt_testcomputingtimev3 downscale4_generate_messages_cpp)

#add_executable(torqueopt_testcomputingtimev4 src/torqueopt_testcomputingtimev4.cpp)
#target_link_libraries(torqueopt_testcomputingtimev4 ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})
#add_dependencies(torqueopt_testcomputingtimev4 downscale4_generate_messages_cpp)

#add_executable(torqueopt_testcomputingtimev5 src/torqueopt_testcomputingtimev5.cpp)
#target_link_libraries(torqueopt_testcomputingtimev5 ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})
#add_dependencies(torqueopt_testcomputingtimev5 downscale4_generate_messages_cpp)

#add_executable(newtorqueopt_v1 src/newtorqueopt_v1.cpp)
#target_link_libraries(newtorqueopt_v1 ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})
#add_dependencies(newtorqueopt_v1 downscale4_generate_messages_cpp)

#add_executable(newtorqueoptcircular_v1 src/newtorqueoptcircular_v1.cpp)
#target_link_libraries(newtorqueoptcircular_v1 ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})
#add_dependencies(newtorqueoptcircular_v1 downscale4_generate_messages_cpp)

#add_executable(newtorqueopt_v2 src/newtorqueopt_v2.cpp)
#target_link_libraries(newtorqueopt_v2 ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})
#add_dependencies(newtorqueopt_v2 downscale4_generate_messages_cpp)

#add_executable(newtorqueopt_v3 src/newtorqueopt_v3.cpp)
#target_link_libraries(newtorqueopt_v3 ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})
#add_dependencies(newtorqueopt_v3 downscale4_generate_messages_cpp)

#add_executable(newtorqueopt_v4 src/newtorqueopt_v4.cpp)
#target_link_libraries(newtorqueopt_v4 ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})
#add_dependencies(newtorqueopt_v4 downscale4_generate_messages_cpp)

#add_executable(newtorqueopt_v5 src/newtorqueopt_v5.cpp)
#target_link_libraries(newtorqueopt_v5 ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})
#add_dependencies(newtorqueopt_v5 downscale4_generate_messages_cpp)

#add_executable(newtorqueoptcircular_v2 src/newtorqueoptcircular_v2.cpp)
#target_link_libraries(newtorqueoptcircular_v2 ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})
#add_dependencies(newtorqueoptcircular_v2 downscale4_generate_messages_cpp)

#add_executable(newtorqueoptcircular_v3 src/newtorqueoptcircular_v3.cpp)
#target_link_libraries(newtorqueoptcircular_v3 ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})
#add_dependencies(newtorqueoptcircular_v3 downscale4_generate_messages_cpp)

#add_executable(newtorqueoptcircular_v4 src/newtorqueoptcircular_v4.cpp)
#target_link_libraries(newtorqueoptcircular_v4 ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})
#add_dependencies(newtorqueoptcircular_v4 downscale4_generate_messages_cpp)

#add_executable(torqueoptwithdata src/torqueoptwithdata.cpp)
#target_link_libraries(torqueoptwithdata ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})
#add_dependencies(torqueoptwithdata downscale4_generate_messages_cpp)

add_executable(codequartercircle src/codequartercircle.cpp)
target_link_libraries(codequartercircle ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(codequartercircle downscale4_generate_messages_cpp)

add_executable(codestraightline src/codestraightline.cpp)
target_link_libraries(codestraightline ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(codestraightline downscale4_generate_messages_cpp)

add_executable(teleoperation src/teleoperation.cpp)
target_link_libraries(teleoperation ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(teleoperation downscale4_generate_messages_cpp)

#add_executable(cartesian_stiffness_hammering src/cartesian_stiffness_hammering.cpp)
#target_link_libraries(cartesian_stiffness_hammering ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES}PowerPMACcontrol libssh2Driver ssh2 argParser -lpthread)
#add_dependencies(cartesian_stiffness_hammering downscale4_generate_messages_cpp)

#add_executable(tele_hammering src/tele_hammering.cpp)
#target_link_libraries(tele_hammering ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES}PowerPMACcontrol libssh2Driver ssh2 argParser -lpthread)
#add_dependencies(tele_hammering downscale4_generate_messages_cpp)

#add_executable(sfeopt src/sfeopt.cpp)
#target_link_libraries(sfeopt ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES}PowerPMACcontrol libssh2Driver ssh2 argParser -lpthread)
#add_dependencies(sfeopt downscale4_generate_messages_cpp)

#add_executable(tele_hammering_realrobot src/tele_hammering_realrobot.cpp)
#target_link_libraries(tele_hammering_realrobot ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES}PowerPMACcontrol libssh2Driver ssh2 argParser -lpthread)
#add_dependencies(tele_hammering_realrobot downscale4_generate_messages_cpp)
