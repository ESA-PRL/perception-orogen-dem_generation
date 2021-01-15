PROJECT(dem_generation)
cmake_minimum_required(VERSION 2.6)

find_package(PCL 1.7 REQUIRED COMPONENTS common)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_definitions(-Wall)
add_definitions(-DBASE_LOG_NAMESPACE=${PROJECT_NAME})
add_definitions(-DBASE_LOG_WARN)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

SET (CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/.orogen/config")
INCLUDE(dem_generationBase)
