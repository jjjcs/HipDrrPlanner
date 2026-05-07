@echo off
chcp 65001
rem current path
set current_cmake_dir=%~dp0
echo %current_cmake_dir%

mkdir %current_cmake_dir%\Bin\Debug\
mkdir %current_cmake_dir%\Bin\Release\

mkdir %current_cmake_dir%\Lib\Debug\
mkdir %current_cmake_dir%\Lib\Release\

ROBOCOPY %current_cmake_dir%..\..\soups\bin\Release\vtk9.0.1\ %current_cmake_dir%\Bin\Release\ /E /MT:10 /W:3
ROBOCOPY %current_cmake_dir%..\..\soups\bin\Debug\vtk9.0.1\ %current_cmake_dir%\Bin\Debug\ /E /MT:10 /W:3

ROBOCOPY %current_cmake_dir%..\..\soups\bin\Release\itk5.1.2\ %current_cmake_dir%\Bin\Release\ /E /MT:10 /W:3
ROBOCOPY %current_cmake_dir%..\..\soups\bin\Debug\itk5.1.2\ %current_cmake_dir%\Bin\Debug\ /E /MT:10 /W:3

ROBOCOPY %current_cmake_dir%..\..\soups\bin\Release\opencv4.5.0\ %current_cmake_dir%\Bin\Release\ /E /MT:10 /W:3
ROBOCOPY %current_cmake_dir%..\..\soups\bin\Debug\opencv4.5.0\ %current_cmake_dir%\Bin\Debug\ /E /MT:10 /W:3

rem ROBOCOPY %current_cmake_dir%..\..\soups\bin\Release\dcmtk3.6.6\ %current_cmake_dir%\Bin\Release\ *.dll /E /MT:10 /W:3
rem ROBOCOPY %current_cmake_dir%..\..\soups\bin\Debug\dcmtk3.6.6\ %current_cmake_dir%\Bin\Debug\ *.dll /E /MT:10 /W:3

ROBOCOPY %current_cmake_dir%..\..\soups\bin\Release\glog0.7.0\ %current_cmake_dir%\Bin\Release\ *.dll /E /MT:10 /W:3
ROBOCOPY %current_cmake_dir%..\..\soups\bin\Debug\glog0.7.0\ %current_cmake_dir%\Bin\Debug\ *.dll /E /MT:10 /W:3

rem copy "%current_cmake_dir%..\..\build\Bin\Release\MERILCommon.dll" "%current_cmake_dir%Bin\Release\MERILCommon.dll" /y
rem copy "%current_cmake_dir%..\..\build\Bin\Release\MERILCommon.pdb" "%current_cmake_dir%Bin\Release\MERILCommon.pdb" /y
rem copy "%current_cmake_dir%..\..\build\Bin\Debug\MERILCommond.dll" "%current_cmake_dir%Bin\Debug\MERILCommond.dll" /y
rem copy "%current_cmake_dir%..\..\build\Bin\Debug\MERILCommond.pdb" "%current_cmake_dir%Bin\Debug\MERILCommond.pdb" /y

rem ROBOCOPY %current_cmake_dir%..\..\build\Bin\Config %current_cmake_dir%\Bin\Config /E /MT:10 /W:3
rem ROBOCOPY %current_cmake_dir%\Resource %current_cmake_dir%\Bin\Resource /E /MT:10 /W:3

mkdir "Build"
cd "Build"

cmake ../ -G "Visual Studio 16 2019" -DCMAKE_PREFIX_PATH=%DCMAKE_Qt%

pause