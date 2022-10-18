cmake_minimum_required(VERSION 3.10 FATAL_ERROR)

project( stereo_estimation )

###############
# packages #
###############

find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

find_package(OpenMP)

set(Qt5_DIR /opt/homebrew/Cellar/qt@5/5.15.6/lib/cmake/Qt5)
set(VTK_DIR /opt/homebrew/Cellar/vtk/9.2.2/lib/cmake/vtk-9.2)
find_package(VTK REQUIRED)
include(${VTK_USE_FILE})

find_package(PCL 1.3 REQUIRED COMPONENTS common io VISUALIZATION)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

###############
# executables #
###############

add_executable( stereo src/main.cpp )
target_link_libraries( stereo ${OpenCV_LIBS} )
target_link_libraries( stereo ${VTK_LIBRARIES} )
target_link_libraries( stereo ${PCL_LIBRARIES} ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_VISUALIZATION_LIBRARIES} )

if(OpenMP_CXX_FOUND)
    target_link_libraries(stereo OpenMP::OpenMP_CXX)
endif()



