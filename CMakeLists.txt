cmake_minimum_required(VERSION 3.0.2)
project(controller)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  nav_msgs
  tf
)

 add_message_files(
   FILES
   VcuCommandC1.msg
   VcuFeedbackC1.msg
 )

 generate_messages(
   DEPENDENCIES
   std_msgs
   nav_msgs
 )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES conrtoller
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime nav_msgs tf
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include
)

link_directories()

add_executable(VCUfeedback src/VCUfeedback.cpp)
add_executable(IVCU_VCUCmmand src/IVCU_VCUCmmand.cpp)
add_executable(Test_PC2VEH src/Test_PC2VEH.cpp)

 target_link_libraries(VCUfeedback
   ${catkin_LIBRARIES}
 )

 target_link_libraries(IVCU_VCUCmmand
   ${catkin_LIBRARIES}
 )

 target_link_libraries(Test_PC2VEH
   ${catkin_LIBRARIES}
 )
