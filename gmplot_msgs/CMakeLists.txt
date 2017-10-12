cmake_minimum_required(VERSION 2.8.3)
project(gmplot_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
)

add_message_files(
  DIRECTORY
    msg
  FILES
    PlotPoint.msg
)

add_service_files(
  DIRECTORY
    srv
  FILES
    PlotMap.srv
)

generate_messages(
  DEPENDENCIES
)

catkin_package()
