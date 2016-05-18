cmake_minimum_required(VERSION 3.0.2 FATAL_ERROR)

project(pcl_video_player)

## Find packages
find_package(PCL 1.8 REQUIRED)
find_package(Qt5Widgets REQUIRED)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

# Find includes in corresponding build directories
set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# Output to a bin folder in the project source
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_HOME_DIRECTORY}/bin)

include_directories (${PCL_INCLUDE_DIRS})
link_directories    (${PCL_LIBRARY_DIRS})
add_definitions     (${PCL_DEFINITIONS})

set(PROJECT_SOURCES        src/pcd_video_player.cpp)
set(PROJECT_HEADERS        include/pcd_video_player.h)
set(PROJECT_FORMS          forms/pcd_video_player.ui)
set(PROJECT_INCLUDE_DIR    include)

#qt5_wrap_cpp(PROJECT_HEADER_MOC ${PROJECT_HEADERS})
qt5_wrap_ui(PROJECT_UI_MOC ${PROJECT_FORMS})

include_directories(${PROJECT_INCLUDE_DIR})

add_executable(pcl_video_player ${PROJECT_SOURCES} ${PROJECT_HEADERS} ${PROJECT_UI_MOC})
#add_executable(pcl_video_player ${PROJECT_SOURCES} ${PROJECT_HEADERS} ${PROJECT_HEADER_MOC} ${PROJECT_UI_MOC})

target_link_libraries(pcl_video_player ${PCL_LIBRARIES} Qt5::Widgets)


