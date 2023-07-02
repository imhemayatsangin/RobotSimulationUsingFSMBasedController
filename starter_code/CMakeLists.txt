cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(lab9)

# include c++11
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++11")


# compiling mode
set(CMAKE_BUILD_TYPE "Release")

#set the default path for built executables to the "bin" directory
#set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# if you don't want the full compiler output, remove the following line
set(CMAKE_VERBOSE_MAKEFILE ON)

# include pkgconfig
include(FindPkgConfig)

# include PlayerC++ for robot controlling
find_package(PlayerC++)
IF (PKG_CONFIG_FOUND)
    pkg_check_modules (PLAYERCPP playerc++)
    IF(PLAYERCPP_FOUND)
        include_directories(${PLAYERCPP_INCLUDE_DIRS})
        MESSAGE(STATUS "PlayerC++ found")
    ELSE(PLAYERCPP_FOUND)
        MESSAGE(FATAL "PlayerC++ cannot be found")
    ENDIF(PLAYERCPP_FOUND)
ENDIF (PKG_CONFIG_FOUND)

# define the path of source file
include_directories(src/ include/)

# generate executable file for sensoryreadings code
add_executable(robot_follower src/robot_follower.cpp)
target_link_libraries(robot_follower ${PLAYERCPP_LIBRARIES})
