cmake_minimum_required(VERSION 3.1 FATAL_ERROR)
project(command)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED)
find_package(catkin REQUIRED COMPONENTS std_msgs geometry_msgs roscpp)

#listing compiler features
message("Your C++ compiler supports these C++ features:")
foreach(i ${CMAKE_CXX_COMPILE_FEATURES})
  message("${i}")
endforeach()



#collecting all base source files from main directory
aux_source_directory(. SRC_LIST)


#adding header directories
include_directories(${catkin_INCLUDE_DIRS})
include_directories(${INCLUDE_DIRS})




#defining the executable with the source files and linking the GSL library
add_executable(${PROJECT_NAME} ${SRC_LIST})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
target_link_libraries(${PROJECT_NAME} ${LINK_LIBS})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})


