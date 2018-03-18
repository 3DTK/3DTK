:: this script is to build 3dtk on 64bit windows with visual studio 2015
:: if you require support for 32bit windows, please send patches
:: this was tested on Windos 10 64bit

:: To run CL.exe manually from a terminal, you must first setup your
:: environment using a call to
:: C:\Program Files (x86)\Microsoft Visual Studio 12.0\VC\vcvarsall.bat

:: the path where the 3dtk sources are
set sourcedir=%~1

if "%sourcedir%" == "" (
	echo "Usage: %0 sourcedir outdir"
	exit /B 1
)

:: the path where you want the resulting binaries
set outdir=%~2

if "%outdir%" == "" (
	echo "Usage: %0 sourcedir outdir"
	exit /B 1
)

:: the build type (one of Debug, Release, RelWithDebInfo and MinSizeRel)
set buildtype=Release

:: The source directory must not be in the root of a drive letter under
:: windows. See:
::  - http://stackoverflow.com/questions/31871972/
::  - http://public.kitware.com/pipermail/cmake/2015-August/061332.html
::  - http://www.cmake.org/Bug/view.php?id=15134
::  - http://www.cmake.org/Bug/view.php?id=10072
::
:: The output directory must *not* be a network drive or otherwise Windows
:: will refuse to load any dll from it.
::
:: Also, *never* change this file while the script is still running. It seems
:: that on windows, the script is not read into memory and executed by the
:: interpreter but instead read on demand.

if not exist %sourcedir% (
	echo %sourcedir% does not exist. Make sure the sourcedir variable is set to the path of the 3DTK sources.
	exit /B 1
)


if not exist %outdir% (
	echo %outdir% does not exist. Make sure the outdir variable is set to an existing path.
	exit /B 1
)

@echo on

:: FIXME: add cgal once appveyor installs a vcpkg version greater than 0.0.105
:: with https://github.com/Microsoft/vcpkg/pull/2962
vcpkg --triplet x64-windows install ^
	qt5 ^
	libpng ^
	boost ^
	opencv ^
	wxwidgets ^
	eigen3 ^
	python3 ^
	zlib ^
	freeglut ^
	suitesparse

if %ERRORLEVEL% GEQ 1 (
	echo vcpkg install failed
	exit /B 1
)

:: use setlocal to make sure that the directory is only changed for this part
:: of the script and not on the outside
setlocal
:: need /d if %outdir% is a different drive letter than the current working
:: directory
cd /d %outdir%
cmake ^
	-G "Visual Studio 14 2015 Win64" ^
	-DCMAKE_TOOLCHAIN_FILE=c:/tools/vcpkg/scripts/buildsystems/vcpkg.cmake ^
	-DCXSPARSE_INCLUDE_DIR=c:/tools/vcpkg/installed/x64-windows/include/suitesparse ^
	-DwxWidgets_LIB_DIR=c:/tools/vcpkg/installed/x64-windows/lib ^
	-DwxWidgets_INCLUDE_DIRS=c:/tools/vcpkg/installed/x64-windows/include ^
	-DOUTPUT_DIRECTORY:PATH=%outdir% ^
	-DWITH_GLFW=OFF ^
	-DWITH_XMLRPC=OFF ^
	-DWITH_LIBCONFIG=OFF ^
	-DWITH_FTGL=OFF ^
	-DWITH_CGAL=OFF ^
	%sourcedir%

if %ERRORLEVEL% GEQ 1 (
	echo cmake config failed
	exit /B 1
)

cmake --build . --config %buildtype% -- /m

if %ERRORLEVEL% GEQ 1 (
	echo cmake --build failed
	exit /B 1
)
endlocal

echo "build successful!"
