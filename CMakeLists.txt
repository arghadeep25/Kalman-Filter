cmake_minimum_required(VERSION 3.5)

project(kalman_lib DESCRIPTION "Kalman Filter with CPP" LANGUAGES CXX)
set(PROJECT_VERSION_MAJOR 0)
set(PROJECT_VERSION_MINOR 1)
set(PROJECT_VERSION_PATCH 0)
set(PROJECT_VERSION ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}.${PROJECT_VERSION_PATCH})

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
    set(CMAKE_CXX_FLAGS_RELEASE "-O2 -DEIGEN_NO_DEBUG")
endif ()

set(INC_DIR kalman_filter)

find_package(Eigen3 REQUIRED)
if(Eigen3_FOUND)
    message("Eigen Version: ${Eigen3_VERSION} found")
endif()


option(ENABLE_EXAMPLES "Enable examples" ${BUILD_EXAMPLE})
set(BUILD_EXAMPLE ON)

if (ENABLE_EXAMPLES)
    message("Building examples.")
    add_library(kalman_lib INTERFACE)
    target_include_directories(kalman_lib INTERFACE ${INC_DIR})
    target_link_libraries(kalman_lib INTERFACE stdc++fs Eigen3::Eigen)
    add_subdirectory(examples)
endif ()


