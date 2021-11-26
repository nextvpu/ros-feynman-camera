cmake_minimum_required(VERSION 3.10)
project(apitest)
set(CMAKE_CXX_STANDARD 11)

find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})

include_directories(../include /usr/include/libusb-1.0)
set(MY_PROJECT_SRC
        ../src/feynman_sdk.cpp
        ../src/ring_queue.cpp)

add_executable(apitest ${MY_PROJECT_SRC} cppapi.cpp apitest.cpp)

SET(CMAKE_CXX_FLAGS "-p -g")
target_link_libraries(apitest
    ${GTEST_BOTH_LIBRARIES}
    pthread
    usb-1.0)

