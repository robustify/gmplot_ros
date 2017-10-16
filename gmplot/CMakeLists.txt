cmake_minimum_required(VERSION 2.8.3)
project(gmplot)

find_package(catkin REQUIRED COMPONENTS
  rospy
)

catkin_package()

install(
  PROGRAMS
    src/gmplot_service.py
    src/gmplot.py
    src/color_dicts.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
