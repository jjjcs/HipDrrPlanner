# CMake build system for Qt based project
# Filename: CMakeLists.txt
# @author        Yuan Gao
# @version       1.00         
# @Date 		 2024-09-14

cmake_minimum_required(VERSION 3.9.0)
add_compile_options($<$<COMPILE_LANGUAGE:CXX>:$<$<CXX_COMPILER_ID:MSVC>:/MP>>)

# Project Basic Information
project(Application LANGUAGES CXX CUDA)
set(CMAKE_CONFIGURATION_TYPES "Debug;Release")
add_compile_definitions(APPLICATION_EXPORTS)
# reset the root folder to the upper layer of the visual studio 2015 folder
set(ROOT "${PROJECT_SOURCE_DIR}/" CACHE INTERNAL "root folder of source code")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_DEBUG ${ROOT}../Lib/Debug)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_DEBUG ${ROOT}../Lib/Debug)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG ${ROOT}../Bin/Debug)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_RELEASE ${ROOT}../Lib/Release)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_RELEASE ${ROOT}../Lib/Release)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE ${ROOT}../Bin/Release)

set_property(GLOBAL PROPERTY USE_FOLDERS ON)
add_compile_options($<$<COMPILE_LANGUAGE:CXX>:$<$<CXX_COMPILER_ID:MSVC>:/Od>>)
add_compile_options($<$<COMPILE_LANGUAGE:CXX>:$<$<CXX_COMPILER_ID:MSVC>:/Zi>>)
add_link_options(/DEBUG)
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /Zi")
set(CMAKE_SHARED_LINKER_FLAGS_RELEASE "${CMAKE_SHARED_LINKER_FLAGS_RELEASE} /DEBUG /OPT:REF /OPT:ICF")
set(CMAKE_EXE_LINKER_FLAGS_DEBUG "${CMAKE_EXE_LINKER_FLAGS_DEBUG} /DEBUG:FASTLINK" )

# Find includes in the build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_WIN32_EXECUTABLE ON)

# include directories
include_directories( 
		${ROOT}../HipSurgeryPlanner/
		${ROOT}../../../soups/include/opencv4.10.0/
		${ROOT}../../../soups/include/itk5.4.0/
		${ROOT}../../../soups/include/vtk9.0.1/
		#${ROOT}../../../soups/include/eigen3.4.0/eigen3/
				)

# lib directories
link_directories(
		${ROOT}../Lib/$(Configuration)
		${ROOT}../../../soups/lib/$(Configuration)/opencv4.10.0
		${ROOT}../../../soups/lib/$(Configuration)/itk5.4.0
		${ROOT}../../../soups/lib/$(Configuration)/vtk9.0.1
		${ROOT}../../../build/Lib/$(Configuration)/
		${ROOT}../HipSurgeryPlanner/lib/Release
				)	
# Enable Qt resource and object compile
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

file(GLOB RESOURCE_FILES ${ROOT}/*.qrc)
source_group("Resource Files" FILES ${RESOURCE_FILES})

file(GLOB UI_FILES ${ROOT}/*.ui)
source_group("Ui Files" FILES ${UI_FILES})

file(GLOB HEADERS  ${ROOT}*.h)
source_group("Header Files" FILES ${HEADERS})

file(GLOB SRCS ${ROOT}*.cpp)
source_group("Source Files" FILES ${SRCS})

# Find the QtWidgets library
find_package(Qt5 COMPONENTS Widgets Core Gui Network Xml Sql  REQUIRED)
# dont genemrate ui, directly use .h file
QT5_WRAP_UI(WRAP_UI ${UI_FILES})

add_executable(${PROJECT_NAME} ${SRCS} ${HEADERS} ${RESOURCE_FILES} ${CONFIG_FILES} ${WRAP_UI})

set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 17)
set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD_REQIRED ON)
set_property(TARGET ${PROJECT_NAME} PROPERTY VS_DEBUGGER_WORKING_DIRECTORY ${ROOT}../Bin/$(Configuration))

add_dependencies(${PROJECT_NAME} HipSurgeryPlanner) 
#add_definitions(-DWIN32 -DWIN64 -DUNICODE -D_UNICODE -D_WINDOWS)

target_link_libraries( ${PROJECT_NAME} Qt5::Core Qt5::Gui Qt5::Widgets Qt5::Network Qt5::Xml Qt5::Sql	
		${OPENCV_LIB_4100_Release}
		${ITK_LIB_540_Release}
		${VTK_LIB_901_Release}
		optimized HipSurgeryPlanner.lib
		)

target_link_libraries( ${PROJECT_NAME} Qt5::Core Qt5::Gui Qt5::Widgets Qt5::Network Qt5::Xml Qt5::Sql 
		${OPENCV_LIB_4100_Debug}
		${ITK_LIB_540_Debug}
		${VTK_LIB_901_Debug}
		debug HipSurgeryPlannerd.lib
		)		
set_target_properties (${PROJECT_NAME} PROPERTIES DEBUG_POSTFIX d CXX_STANDARD 17  CXX_STANDARD_REQUIRED ON CUDA_ARCHITECTURES "80;89")

add_custom_command(
	TARGET 
		Application 
	POST_BUILD
    COMMAND 
		COMMAND ${CMAKE_COMMAND} -E chdir ${CMAKE_CURRENT_SOURCE_DIR}/.. deploy.cmd
    COMMENT 
		"Running deploy command"
)


