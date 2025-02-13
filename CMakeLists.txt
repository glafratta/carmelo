cmake_minimum_required(VERSION 3.1.0)

project(CloCK LANGUAGES CXX)

include(CTest)
set (CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Debug)

find_package(Threads)
find_package(PkgConfig)
find_package(OpenCV REQUIRED)
#find_package(box2d REQUIRED)


add_subdirectory(src "${CMAKE_CURRENT_BINARY_DIR}/src")



if(${CMAKE_SYSTEM_PROCESSOR} MATCHES "aarch64") # for Raspberry Pi
  find_package(iir)
  pkg_check_modules(LIBCAMERA REQUIRED IMPORTED_TARGET libcamera)
  include_directories(${CMAKE_SOURCE_DIR} ${CMAKE_SOURCE_DIR}/.. ${LIBCAMERA_INCLUDE_DIRS} ${OPENCV_INCLUDE_DIRS})
  include(GNUInstallDirs)
  add_executable(target custom.h target.cpp)
  add_executable(targetless custom.h targetless.cpp)
  #add_executable(targetless_forget custom.h targetless_forget.cpp)
  target_link_libraries(target ${OpenCV_LIBS} iir::iir_static box2d alphabot a1lidarrpi CloCK pigpio PkgConfig::LIBCAMERA cam2opencv)
  target_link_libraries(targetless ${OpenCV_LIBS} iir::iir_static box2d alphabot a1lidarrpi CloCK pigpio PkgConfig::LIBCAMERA cam2opencv)
  #target_link_libraries(targetless_forget ${OpenCV_LIBS} iir::iir_static box2d alphabot a1lidarrpi CloCK pigpio PkgConfig::LIBCAMERA cam2opencv)
  message("detected architecture aarch64, building executables")
else()
#	add_subdirectory(test)
  add_subdirectory(test "${CMAKE_CURRENT_BINARY_DIR}/test")

  add_subdirectory(test/unit_test "${CMAKE_CURRENT_BINARY_DIR}/test/unit_test")

	message("not building any executable")
endif()


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

add_compile_options(-Wall -pedantic)

enable_testing()
