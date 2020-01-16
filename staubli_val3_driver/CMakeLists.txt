cmake_minimum_required(VERSION 2.8.3)
project(staubli_val3_driver)

find_package(catkin REQUIRED)
catkin_package()

if (CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(tests/roslaunch_test.xml)
endif()

install(DIRECTORY launch val3
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(FILES README.md DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
