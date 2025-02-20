cmake_minimum_required(VERSION 3.1.0)

project(CloCK LANGUAGES CXX)

include(CTest)
set (CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Debug)

find_package(Threads)
find_package(PkgConfig)
find_package(OpenCV REQUIRED)


add_subdirectory(src "${CMAKE_CURRENT_BINARY_DIR}/src")



if(${CMAKE_SYSTEM_PROCESSOR} MATCHES "aarch64") # for Raspberry Pi
  include(GNUInstallDirs)
  add_executable(target custom.h target.cpp)
  add_executable(targetless custom.h targetless.cpp)
  target_link_libraries(target ${OpenCV_LIBS} box2d alphabot a1lidarrpi CloCK pigpio)
  target_link_libraries(targetless ${OpenCV_LIBS}box2d alphabot a1lidarrpi CloCK pigpio)
  message("detected architecture aarch64, building executables")
else()
  add_subdirectory(test/unit_test "${CMAKE_CURRENT_BINARY_DIR}/test/unit_test")

	message("not building any executable")
endif()


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

add_compile_options(-Wall -pedantic)

enable_testing()
