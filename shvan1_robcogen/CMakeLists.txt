cmake_minimum_required(VERSION 3.10.2)
project(shvan1_robcogen)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS pronto_quadruped_commons
                                        quadruped_robcogen
                                        shvan1_description)

set(LIB_NAME ${PROJECT_NAME})

find_package(Eigen3)

catkin_package(INCLUDE_DIRS include
               LIBRARIES shvan1_robcogen
               CATKIN_DEPENDS pronto_quadruped_commons
                              quadruped_robcogen 
                              shvan1_description)

include_directories(include 
                    ${catkin_INCLUDE_DIRS})

include_directories(./include)
include_directories(./include/${PROJECT_NAME})
include(${quadruped_robcogen_SOURCE_PREFIX}/cmake/robcogen.cmake)

# call the macro to generate the C++ code
robcogen(shvan1 shvan1_description shvan1)

install(TARGETS ${LIB_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})


