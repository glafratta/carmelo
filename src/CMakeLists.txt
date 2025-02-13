include(GNUInstallDirs)

set (CMAKE_CXX_STANDARD 17)

set(SRC disturbance.cpp configurator.cpp task.cpp graphTools.cpp measurement.cpp worldbuilder.cpp sensor.cpp debug.cpp)

set(HEAD disturbance.h configurator.h robot.h task.h graphTools.h affordance.h settings.h measurement.h worldbuilder.h sensor.h debug.h)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(OpenCV REQUIRED)

find_package(box2d REQUIRED)

find_package(PkgConfig)

add_library(CloCK ${SRC} ${HEAD})

if(box2d_FOUND)
        target_link_libraries(CloCK box2d ${OpenCV_LIBS} pthread)
        message("box2d installed ${BOX2D_LIBS}")
else()
      set(BOX2D_BIN_DIR ${"/usr/local/lib/aarch64-linux-gnu"})
       target_link_libraries(HICS ${BOX2D_BIN_DIR}/libbox2d.a ${OpenCV_LIBS} pthread)
       #message("fetching static library from ${BOX2D_BIN_DIR}")
	message("BOX2D NOT INSTALLED")
endif()
set_target_properties(CloCK PROPERTIES POSITION_INDEPENDENT_CODE TRUE 
PUBLIC_HEADER "${HEAD}")

install(TARGETS CloCK 
	EXPORT CloCK-targets 
	LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
	ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
	PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

message("library and archive destination " ${CMAKE_INSTALL_LIBDIR})

set(CMAKE_BUILD_TYPE Debug)

add_compile_options(-Wall -pedantic)
