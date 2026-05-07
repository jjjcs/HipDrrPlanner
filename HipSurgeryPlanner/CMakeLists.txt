# CMake build system for Qt based project
# Filename: CMakeLists.txt
# @author        Yuan Gao
# @version       1.00         
# @Date 		 2024-09-26

cmake_minimum_required(VERSION 3.9.0)
add_compile_options($<$<COMPILE_LANGUAGE:CXX>:$<$<CXX_COMPILER_ID:MSVC>:/MP>>)

# Project Basic Information
project(HipSurgeryPlanner LANGUAGES CXX CUDA)
set(CMAKE_CONFIGURATION_TYPES "Debug;Release")
add_compile_definitions(HIPSURGERYPLANNER_EXPORTS)
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
		${ROOT}../../../soups/include/opencv4.10.0/
		${ROOT}../../../soups/include/itk5.4.0/
		${ROOT}../../../soups/include/vtk9.0.1/
		#${ROOT}../../../soups/include/eigen3.4.0/eigen3/
			)

# lib directories
link_directories(
		${ROOT}../../../build/Lib/$(Configuration)/
		${ROOT}../../../soups/lib/$(Configuration)/opencv4.10.0
		${ROOT}../../../soups/lib/$(Configuration)/itk5.4.0
		${ROOT}../../../soups/lib/$(Configuration)/vtk9.0.1
			)
			
# Enable Qt resource and object compile
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

#file(GLOB RESOURCE_FILES ${ROOT}Resource/*.qrc)
#source_group("Resource Files/Resource" FILES ${RESOURCE_FILES})

#file(GLOB UI_FILES ${ROOT}/*.ui)
source_group("Ui Files" FILES ${UI_FILES})

file(GLOB HEADERS  ${ROOT}*.h  ${ROOT}*.hxx ${ROOT}*.cuh)
source_group("Header Files" FILES ${HEADERS})
file(GLOB SRCS ${ROOT}*.cpp ${ROOT}*.cu)
source_group("Source Files" FILES ${SRCS})

# Find the QtWidgets library
find_package(Qt5 COMPONENTS Widgets Core Gui Network Xml Sql  REQUIRED)
# generate ui
QT5_WRAP_UI(WRAP_UI ${UI_FILES} ${UI_COMMONWIDGETS_FILES})

add_library(${PROJECT_NAME} SHARED 
${SRCS} ${HEADERS} ${RESOURCE_FILES} ${WRAP_UI} )

set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 17)
set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD_REQIRED ON)

#add_dependencies(${PROJECT_NAME} CommonFunction) 
#add_definitions(-DWIN32 -DWIN64 -DUNICODE -D_UNICODE -D_WINDOWS)
target_link_libraries( ${PROJECT_NAME} Qt5::Core Qt5::Gui Qt5::Widgets Qt5::Network Qt5::Xml Qt5::Sql
		${OPENCV_LIB_4100_Release}
		${ITK_LIB_540_Release}	
		${VTK_LIB_901_Release}
		CUDA::cudart	
		CUDA::cublas
		)

target_link_libraries( ${PROJECT_NAME} Qt5::Core Qt5::Gui Qt5::Widgets Qt5::Network Qt5::Xml Qt5::Sql  	
		${OPENCV_LIB_4100_Debug}
		${ITK_LIB_540_Debug}
		${VTK_LIB_901_Debug}
		CUDA::cudart
		CUDA::cublas
		)	
set_target_properties (${PROJECT_NAME} PROPERTIES
		DEBUG_POSTFIX d 
		CUDA_ARCHITECTURES "80;89"
 		CUDA_RUNTIME_LIBRARY "Shared"
   		CUDA_SEPARABLE_COMPILATION OFF
)
execute_process(
    COMMAND nvidia-smi --query-gpu=compute_cap --format=csv,noheader
    OUTPUT_VARIABLE GPU_ARCHS
    OUTPUT_STRIP_TRAILING_WHITESPACE
)


target_compile_options(HipSurgeryPlanner
	PRIVATE
	#$<$<CXX_COMPILER_ID:MSVC>:/bigobj>
	$<$<COMPILE_LANGUAGE:CXX>:$<$<CXX_COMPILER_ID:MSVC>:/bigobj>>
 )
