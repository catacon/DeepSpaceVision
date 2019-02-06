cmake_minimum_required(VERSION 3.0.0)
project(DeepSpaceVision VERSION 0.1.0)

find_package(OpenCV REQUIRED)

## load in pkg-config support
find_package(PkgConfig)
## use pkg-config to get hints for 0mq locations
pkg_check_modules(PC_ZeroMQ QUIET zmq)

## use the hint from above to find where 'zmq.hpp' is located
find_path(ZeroMQ_INCLUDE_DIR
        NAMES zmq.hpp
        PATHS ${PC_ZeroMQ_INCLUDE_DIRS}
        )

## use the hint from about to find the location of libzmq
find_library(ZeroMQ_LIBRARY
        NAMES zmq
        PATHS ${PC_ZeroMQ_LIBRARY_DIRS}
        )

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/libs/include)

add_executable(DeepSpaceVision main.cpp Setup.cpp DeepSpaceVision.cpp DeepSpaceProcessor.cpp TargetFinder.cpp DataSender.cpp)

target_link_libraries(DeepSpaceVision ${OpenCV_LIBS} pthread zmq)

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
