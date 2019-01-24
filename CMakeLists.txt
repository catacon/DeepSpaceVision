cmake_minimum_required(VERSION 3.0.0)
project(DeepSpaceVision VERSION 0.1.0)

find_package(OpenCV REQUIRED)
find_package(ZeroMQ REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/libs/include)

add_executable(DeepSpaceVision main.cpp Setup.cpp DeepSpaceVision.cpp TargetFinder.cpp DataSender.cpp)

target_link_libraries(DeepSpaceVision ${OpenCV_LIBS} pthread zmq)

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
