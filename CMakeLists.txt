cmake_minimum_required(VERSION 3.20)
project(ContingencyPlanning LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type" FORCE)
endif()
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

option(ENABLE_WARNINGS "Enable strict compiler warnings" OFF)

# Dependencies
find_package(spdlog REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(OsqpEigen REQUIRED)
find_package(Protobuf REQUIRED)
find_package(BLAS REQUIRED)
find_package(Threads REQUIRED)
find_package(Eigen3 REQUIRED)

# Foxglove SDK (headers + prebuilt static lib)
set(FOXGLOVE_SOURCE_DIR "${PROJECT_SOURCE_DIR}/3rdparty/foxglove")
file(GLOB FOXGLOVE_SOURCES CONFIGURE_DEPENDS
    "${FOXGLOVE_SOURCE_DIR}/src/*.cpp"
    "${FOXGLOVE_SOURCE_DIR}/src/server/*.cpp"
)

add_library(foxglove_sdk STATIC IMPORTED)
set_target_properties(foxglove_sdk PROPERTIES
    IMPORTED_LOCATION "${FOXGLOVE_SOURCE_DIR}/lib/libfoxglove.a"
    INTERFACE_INCLUDE_DIRECTORIES "${FOXGLOVE_SOURCE_DIR}/include"
)

# Protobuf: generate sources
set(PROTO_FILES "${PROJECT_SOURCE_DIR}/common/protos/planning_info.proto")
set(PROTO_OUTPUT_DIR "${PROJECT_SOURCE_DIR}/common/protos")
get_filename_component(PROTO_FILE_WE ${PROTO_FILES} NAME_WE)
set(PROTO_SRCS "${PROTO_OUTPUT_DIR}/${PROTO_FILE_WE}.pb.cc")
set(PROTO_HDRS "${PROTO_OUTPUT_DIR}/${PROTO_FILE_WE}.pb.h")

add_custom_command(
    OUTPUT ${PROTO_SRCS} ${PROTO_HDRS}
    COMMAND ${Protobuf_PROTOC_EXECUTABLE}
    ARGS --cpp_out=${PROTO_OUTPUT_DIR} --proto_path=${PROTO_OUTPUT_DIR} ${PROTO_FILES}
    DEPENDS ${PROTO_FILES}
    COMMENT "Generating C++ from ${PROTO_FILES}"
    VERBATIM
)

# Sources
file(GLOB SIMULATOR_SOURCES CONFIGURE_DEPENDS "simulator/*.cpp")
file(GLOB PLANNING_SOURCES  CONFIGURE_DEPENDS "planning/*.cpp")
file(GLOB DEC_SOLVER_LIB    CONFIGURE_DEPENDS "3rdparty/control_tree_solver/LIB/*.a")

# Simulator library
add_library(simulator SHARED ${SIMULATOR_SOURCES})
target_compile_definitions(simulator PUBLIC PROJECT_SOURCE_DIR="${PROJECT_SOURCE_DIR}")
target_include_directories(simulator
    PUBLIC
        ${PROJECT_SOURCE_DIR}
        ${FOXGLOVE_SOURCE_DIR}/include
)

# Planning library
add_library(planning SHARED ${PLANNING_SOURCES} ${PROTO_SRCS})
target_include_directories(planning
    PUBLIC
        ${PROJECT_SOURCE_DIR}
        ${PROTO_OUTPUT_DIR}
        ${FOXGLOVE_SOURCE_DIR}/include
        ${PROJECT_SOURCE_DIR}/3rdparty
        ${PROJECT_SOURCE_DIR}/3rdparty/control_tree_solver/Core
        ${PROJECT_SOURCE_DIR}/3rdparty/control_tree_solver/Optim
        ${PROJECT_SOURCE_DIR}/3rdparty/control_tree_solver/OptimDec
)

target_link_libraries(planning
    PUBLIC
        OsqpEigen::OsqpEigen
        ${Protobuf_LIBRARIES}
        ${DEC_SOLVER_LIB}
        ${BLAS_LIBRARIES}
)

# Executable
add_executable(planning_node planning_node.cpp)
target_sources(planning_node PRIVATE ${FOXGLOVE_SOURCES})
target_link_libraries(planning_node
    PRIVATE
        simulator
        planning
        spdlog::spdlog
        yaml-cpp
        foxglove_sdk
        Threads::Threads
        dl
)

if(ENABLE_WARNINGS)
    foreach(tgt simulator planning planning_node)
        target_compile_options(${tgt} PRIVATE -Wall -Wextra -Wpedantic)
    endforeach()
endif()
