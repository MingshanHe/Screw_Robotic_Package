cmake_minimum_required( VERSION 3.21 )
project( Screw_Robotics_Package )

add_compile_options(-std=c++11)

set( CMAKE_BUILD_TYPE "Release" )
set( CMAKE_CXX_FLAGS "-O3" )

include_directories(/usr/local/include/eigen3)

add_library(Screw_Robotics SHARED
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Screw_Robotics.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/Screw_Robotics.h)

add_executable(Test
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Test.cpp)

target_link_libraries(Test Screw_Robotics)