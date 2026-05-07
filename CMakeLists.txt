# CMake build system for Qt based project
# Filename: CMakeLists.txt
# @author        Yuan Gao
# @version       1.00         
# @Date 		 2024-09-20

cmake_minimum_required(VERSION 3.9.0)
add_compile_options($<$<CXX_COMPILER_ID:MSVC>:/MP>)

# Project Basic Information
project(HipSurgeryPlanner LANGUAGES CXX)
set(CMAKE_CONFIGURATION_TYPES "Debug;Release")

# reset the root folder to the upper layer of the visual studio 2017 folder
set(ROOT "${PROJECT_SOURCE_DIR}/" CACHE INTERNAL "root folder of source code")

# add vtk libs
include(${CMAKE_SOURCE_DIR}/VTK_LIBS.cmake)

# add itk libs
include(${CMAKE_SOURCE_DIR}/ITK_LIBS.cmake)

# add itk libs
include(${CMAKE_SOURCE_DIR}/OPENCV_LIBS.cmake)

ADD_SUBDIRECTORY(Application)
ADD_SUBDIRECTORY(HipSurgeryPlanner)

