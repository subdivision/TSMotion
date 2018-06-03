# Created by the script cgal_create_CMakeLists
# This is the CMake script for compiling a set of CGAL applications.

project(MotionPlanningForTwoRobots)

cmake_minimum_required(VERSION 2.8.11)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

# CGAL and its components
find_package(CGAL QUIET COMPONENTS)

if (NOT CGAL_FOUND)

    message(STATUS "This project requires the CGAL library, and will not be compiled.")
    return()

endif ()

# include helper file
include(${CGAL_USE_FILE})


# Boost and its components
find_package(Boost REQUIRED)

if (NOT Boost_FOUND)

    message(STATUS "This project requires the Boost library, and will not be compiled.")

    return()

endif ()

# include for local directory

# include for local package


# Creating entries for target: executable
# ############################

set(SOURCE_FILES ConfigCheck.h ConfigCheck.cpp TSPathFinder.h TSPathFinder.cpp main.cpp CGAL_defines.h)
add_executable(MotionPlanningForTwoRobots ${SOURCE_FILES})

add_to_cached_list(CGAL_EXECUTABLE_TARGETS MotionPlanningForTwoRobots)

# Link the executable to CGAL and third-party libraries
target_link_libraries(MotionPlanningForTwoRobots ${CGAL_LIBRARIES} ${CGAL_3RD_PARTY_LIBRARIES})
