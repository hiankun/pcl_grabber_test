cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(PCL_openni_viewer)

find_package(PCL 1.7 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

## OpenNI 2
#option(WITH_OPENNI2 "OpenNI 2 driver support" TRUE)
#if(WITH_OPENNI2)
#    find_package(OpenNI2)
#    if (OPENNI2_FOUND)
#        set(HAVE_OPENNI2 ON)
#        include_directories(SYSTEM ${OPENNI2_INCLUDE_DIRS})
#    endif(OPENNI2_FOUND)
#endif(WITH_OPENNI2)

set(PCL_BUILD_TYPE Release)

file(GLOB PCL_openni_viewer_SRC
    "src/*.h"
    "src/*.cpp"
    )
add_executable(openniViewer ${PCL_openni_viewer_SRC})

target_link_libraries(openniViewer ${PCL_LIBRARIES})
