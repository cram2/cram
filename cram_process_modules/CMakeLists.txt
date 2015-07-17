cmake_minimum_required(VERSION 2.8.3)
project(cram_process_modules)

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

install (FILES cram-process-modules.asd cram-process-modules-test.asd
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
