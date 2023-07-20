cmake_minimum_required(VERSION 3.0.2)
project(antik)

find_package(catkin REQUIRED) 

catkin_package()

foreach(dir
    src)
  install(DIRECTORY ${dir}
      DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
      PATTERN ".svn" EXCLUDE
      PATTERN ".git" EXCLUDE)
endforeach()
