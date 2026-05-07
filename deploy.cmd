@echo off
rem current path
set current_cmake_dir=%~dp0
echo current_cmake_dir

mkdir %current_cmake_dir%..\..\build\Bin\Release\
mkdir %current_cmake_dir%..\..\build\Bin\Debug\

mkdir %current_cmake_dir%..\..\build\Lib\Release\
mkdir %current_cmake_dir%..\..\build\Lib\Debug\

mkdir %current_cmake_dir%..\..\build\Include\HipSurgeryPlanner\

copy "%current_cmake_dir%HipSurgeryPlanner\*.h"  "%current_cmake_dir%..\..\build\Include\HipSurgeryPlanner\" /y

copy "%current_cmake_dir%Bin\Release\HipSurgeryPlanner.dll"  "%current_cmake_dir%..\..\build\Bin\Release\HipSurgeryPlanner.dll" /y
copy "%current_cmake_dir%Bin\Release\HipSurgeryPlanner.pdb"  "%current_cmake_dir%..\..\build\Bin\Release\HipSurgeryPlanner.pdb" /y
copy "%current_cmake_dir%Lib\Release\HipSurgeryPlanner.lib"  "%current_cmake_dir%..\..\build\Lib\Release\HipSurgeryPlanner.lib" /y

copy "%current_cmake_dir%Bin\Debug\HipSurgeryPlannerd.dll"  "%current_cmake_dir%..\..\build\Bin\Debug\HipSurgeryPlannerd.dll" /y
copy "%current_cmake_dir%Bin\Debug\HipSurgeryPlannerd.pdb"  "%current_cmake_dir%..\..\build\Bin\Debug\HipSurgeryPlannerd.pdb" /y
copy "%current_cmake_dir%Lib\Debug\HipSurgeryPlannerd.lib"  "%current_cmake_dir%..\..\build\Lib\Debug\HipSurgeryPlannerd.lib" /y

pause