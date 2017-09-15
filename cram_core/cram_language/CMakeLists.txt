cmake_minimum_required(VERSION 2.8.3)
project(cram_language)

find_package(catkin REQUIRED) 

catkin_package()

foreach(dir
    src
    tests)
  install(DIRECTORY ${dir}
      DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
      PATTERN ".svn" EXCLUDE
      PATTERN ".git" EXCLUDE)
endforeach()

install (FILES cram-language.asd cram-language-tests.asd mainpage.dox
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
