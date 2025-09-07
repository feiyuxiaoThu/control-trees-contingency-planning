cmake_minimum_required(VERSION 3.20)
project(ContingencyPlanning LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose the type of build (Debug or Release)" FORCE)
endif ()

message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

if (CMAKE_BUILD_TYPE STREQUAL "Debug")
    message(STATUS "Configuring Debug build")
    set(CMAKE_BUILD_TYPE "Debug")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -O0")
elseif (CMAKE_BUILD_TYPE STREQUAL "Release")
    message(STATUS "Configuring Release build")
    set(CMAKE_BUILD_TYPE "Release")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -O3")
endif ()

find_package(spdlog REQUIRED)

include_directories("/usr/include/eigen3")
include_directories(${PROJECT_SOURCE_DIR})
include_directories(${PROJECT_SOURCE_DIR}/3rdparty)
include_directories(${PROJECT_SOURCE_DIR}/3rdparty/foxglove/include)

# Find all Foxglove SDK source files
set(foxglove_SOURCE_DIR "${PROJECT_SOURCE_DIR}/3rdparty/foxglove")
file(GLOB FOXGLOVE_SOURCES CONFIGURE_DEPENDS
        "${foxglove_SOURCE_DIR}/src/*.cpp"
        "${foxglove_SOURCE_DIR}/src/server/*.cpp"
)

file(GLOB SIMULATOR_SOURCES CONFIGURE_DEPENDS "simulator/*.cpp")
file(GLOB PLANNING_SOURCES CONFIGURE_DEPENDS "planning/*.cpp")

add_library(simulator SHARED ${SIMULATOR_SOURCES})
add_library(planning SHARED ${PLANNING_SOURCES})
target_link_libraries(planning PUBLIC /usr/local/lib/libosqp.so)

add_executable(planning_node planning_node.cpp)
target_sources(planning_node PRIVATE ${FOXGLOVE_SOURCES})
target_link_libraries(planning_node PRIVATE
    simulator
    planning
    spdlog::spdlog
    ${foxglove_SOURCE_DIR}/lib/libfoxglove.a
    pthread
    dl
)
