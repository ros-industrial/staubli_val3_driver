cmake_minimum_required(VERSION 2.8.3)

project(staubli_val3_driver)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY launch val3
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(FILES README.md DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
