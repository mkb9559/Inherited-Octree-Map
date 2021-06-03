cmake_minimum_required(VERSION 2.8)
project(octree)
#set(OpenCV_DIR /usr/local/opencv341/share/OpenCV)

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()

# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
   add_definitions(-DCOMPILEDWITHC11)
   message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   add_definitions(-DCOMPILEDWITHC0X)
   message(STATUS "Using flag -std=c++0x.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()



find_package (catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  tf
  geometry_msgs
)

find_package(Boost REQUIRED COMPONENTS system)
find_package(Pangolin REQUIRED)
catkin_package(
    INCLUDE_DIRS include
    LIBRARIES
    CATKIN_DEPENDS geometry_msgs roscpp
    DEPENDS Boost
)
include_directories(include
    ${catkin_INCLUDE_DIRS}
    ${Pangolin_INCLUDE_DIRS}
)


add_executable(octree 
    src/main.cpp
    src/inherit.cpp
    src/Navigation3D.cpp
    src/ocio.cpp
    src/octree.cpp
    src/Point3D.cpp
    src/Point3DNiv.cpp
    src/treenode.cpp
)
target_link_libraries(octree
    ${catkin_LIBRARIES}
    ${Pangolin_LIBRARIES}
    boost_system
    boost_filesystem
)












