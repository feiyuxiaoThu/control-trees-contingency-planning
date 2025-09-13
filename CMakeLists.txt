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
find_package(OsqpEigen REQUIRED)
find_package(Protobuf REQUIRED)

include_directories("/usr/include/eigen3")
include_directories(${PROJECT_SOURCE_DIR})
include_directories(${PROJECT_SOURCE_DIR}/3rdparty)
include_directories(${PROJECT_SOURCE_DIR}/3rdparty/foxglove/include)

# Protobuf settings
set(PROTO_FILES "${PROJECT_SOURCE_DIR}/common/protos/planning_info.proto")
set(PROTO_OUTPUT_DIR "${PROJECT_SOURCE_DIR}/common/protos")

# Define output files
get_filename_component(PROTO_FILE_WE ${PROTO_FILES} NAME_WE)
set(PROTO_SRCS "${PROTO_OUTPUT_DIR}/${PROTO_FILE_WE}.pb.cc")
set(PROTO_HDRS "${PROTO_OUTPUT_DIR}/${PROTO_FILE_WE}.pb.h")

# Custom command to generate protobuf files directly in target directory
add_custom_command(
    OUTPUT ${PROTO_SRCS} ${PROTO_HDRS}
    COMMAND ${Protobuf_PROTOC_EXECUTABLE}
    ARGS --cpp_out=${PROTO_OUTPUT_DIR} --proto_path=${PROTO_OUTPUT_DIR} ${PROTO_FILES}
    DEPENDS ${PROTO_FILES}
    COMMENT "Generating C++ code from ${PROTO_FILES}"
    VERBATIM
)

# Add generated proto directory to includes
include_directories(${PROTO_OUTPUT_DIR})

# Find all Foxglove SDK source files
set(foxglove_SOURCE_DIR "${PROJECT_SOURCE_DIR}/3rdparty/foxglove")
file(GLOB FOXGLOVE_SOURCES CONFIGURE_DEPENDS
        "${foxglove_SOURCE_DIR}/src/*.cpp"
        "${foxglove_SOURCE_DIR}/src/server/*.cpp"
)

file(GLOB SIMULATOR_SOURCES CONFIGURE_DEPENDS "simulator/*.cpp")
file(GLOB PLANNING_SOURCES CONFIGURE_DEPENDS "planning/*.cpp")

add_library(simulator SHARED ${SIMULATOR_SOURCES})
add_library(planning SHARED ${PLANNING_SOURCES} ${PROTO_SRCS})
target_link_libraries(planning PUBLIC OsqpEigen::OsqpEigen ${Protobuf_LIBRARIES})

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
