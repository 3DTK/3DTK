@echo off
:: this script is to build 3dtk on 64bit windows with visual studio 2013
:: if you require support for 32bit windows, please send patches
:: this was tested on Windos 7 64bit

:: To run CL.exe manually from a terminal, you must first setup your
:: environment using a call to
:: C:/Program Files (x86)/Microsoft Visual Studio 12.0/VC/vcvarsall.bat

:: Also make sure that you've installed vcpkg under C:\tools\vcpkg.
:: If you haven't, you can just move it there.

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

:: This script must have the extension .cmd under windows or otherwise (if
:: it's named .bat for example) %ERRORLEVEL% will not be reset but keep being
:: false even if executions of commands succeed.
::
:: This script uses embedded powershell to download files and check their md5
:: sums. The script is not written in powershell itself to avoid stupid popups
:: about this script being insecure. I wonder why there are no popups if
:: powershell is used from within a batch file like this script does...
::
:: Because of powershell, this script needs either Windows 7 or an earlier
:: Windows version with powershell (>= 5.0) installed. The powershell version
:: requirement comes from the Expand-Archive function.
::
:: This script hardcodes the visual studio version. This is because MSVC 14.0
:: is the last version with downloadable binaries available for boost and
:: opencv.
::
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

set cmakedir=%outdir%/3rdparty/cmake/

set cmakeexe=%outdir%/3rdparty/cmake/cmake-3.12.4-win64-x64/bin/cmake.exe
set cmakezip=%outdir%/cmake-3.12.4-win64-x64.zip
set cmakeurl=https://cmake.org/files/v3.12/cmake-3.12.4-win64-x64.zip
set cmakehash=f4-e8-13-07-8f-51-80-aa-ee-a4-5a-5b-87-5b-16-97

if not exist %cmakedir% (
	if not exist %cmakezip% (
		echo downloading %cmakezip%...
		call:download %cmakeurl% %cmakezip%
	)
	echo checking md5sum of %cmakezip%...
	call:checkmd5 %cmakezip% %cmakehash%
	if ERRORLEVEL 1 (
		echo md5sum mismatch
		exit /B 1
	)
	echo extracting %cmakezip% into %cmakedir%...
	call:unzip %cmakezip% %cmakedir%
	if %ERRORLEVEL% GEQ 1 (
		echo cmake unzip failed
		exit /B 1
	)
)

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

:: use setlocal to make sure that the directory is only changed for this part
:: of the script and not on the outside
setlocal
:: need /d if %outdir% is a different drive letter than the current working
:: directory
cd /d %outdir%
"%cmakeexe%" ^
	-G "Visual Studio 15 2017 Win64" ^
	-D CMAKE_TOOLCHAIN_FILE=C:/3dtk/3rdparty/vcpkg/scripts/buildsystems/vcpkg.cmake ^
	-D CXSPARSE_INCLUDE_DIRS=C:/3dtk/3rdparty/vcpkg/packages/suitesparse_x64-windows/include/suitesparse ^
	-D CXSPARSE_LIBRARIES=C:/3dtk/3rdparty/vcpkg/packages/suitesparse_x64-windows/lib/libcxsparse.lib ^
	-D OUTPUT_DIRECTORY:PATH=%outdir% ^
	-D WITH_LIBCONFIG=OFF ^
	-D WITH_CGAL=OFF ^
	-D WITH_LIBZIP=OFF ^
	-D WITH_PYTHON=OFF ^
	-D WITH_APRILTAG=OFF ^
	-D WITH_LASLIB=OFF ^
	-D WITH_WXWIDGETS=OFF ^
	%sourcedir%

if %ERRORLEVEL% GEQ 1 (
	echo cmake config failed
	exit /B 1
)

"%cmakeexe%" --build . --config %buildtype% -- /m

if %ERRORLEVEL% GEQ 1 (
	echo cmake --build failed
	exit /B 1
)
endlocal

echo "build successful!"

goto:eof

:checkmd5
	powershell -command ^
		"$expectedhash = """"%~2"""";" ^
		"$md5 = New-Object -TypeName System.Security.Cryptography.MD5CryptoServiceProvider;" ^
		"$stream = [System.IO.File]::Open(""""%~1"""",[System.IO.Filemode]::Open, [System.IO.FileAccess]::Read);" ^
		"$hash = [System.BitConverter]::ToString($md5.ComputeHash($stream));" ^
		"$stream.Close();" ^
		"if($hash.ToLower().CompareTo($expectedhash.ToLower())){;" ^
		"	exit 1" ^
		"} else {" ^
		"	exit 0" ^
		"}"
	if %ERRORLEVEL% GEQ 1 (
		exit /B 1
	)
	exit /B 0

:download
	powershell -command ^
		"[System.Net.ServicePointManager]::SecurityProtocol = [System.Net.SecurityProtocolType]::Tls12;" ^
		"$wc = New-Object System.Net.WebClient;" ^
		"$wc.DownloadFile(""""%~1""", ^
			"""%~2"""")";
	if %ERRORLEVEL% GEQ 1 (
		exit /B 1
	)
	exit /B 0

:unzip
	powershell -command ^
		"Expand-Archive """"%~1"""" -DestinationPath """"%~2"""""
	if %ERRORLEVEL% GEQ 1 (
		exit /B 1
	)
	exit /B 0

