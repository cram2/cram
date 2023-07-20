cmake_minimum_required(VERSION 3.0.2)
project(cram_math)

find_package(catkin REQUIRED) 

catkin_package()

foreach(dir
    src
    test)
  install(DIRECTORY ${dir}
      DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
      PATTERN ".svn" EXCLUDE
      PATTERN ".git" EXCLUDE)
endforeach()

install (FILES cram-math.asd cram-math-test.asd 
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
