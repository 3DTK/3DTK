@echo off
:: this script is to build 3dtk on 64bit windows with visual studio 2017
:: if you require support for 32bit windows, please send patches
:: this was tested on Windos 7 64bit
::
:: The following components are required:
::    - VC++ 2017 version 15 v141 tools
::    - Windows 10 SDK
::    - Visual C++ ATL
::    - Visual C++ MFC
::    - Visual Studio English Language Pack

:: windows batch induces many headaches... best reference: https://ss64.com/nt/

:: In windows batch, all variables inside an if block or a for loop are
:: expanded *before* the block is run. This means %variables% cannot be
:: updated within an if block or for loop. The following allows to use
:: the !variable! syntax which will expand variables later.
setlocal ENABLEDELAYEDEXPANSION

:: To run CL.exe manually from a terminal, you must first setup your
:: environment using a call to
:: C:\Program Files (x86)\Microsoft Visual Studio 12.0\VC\vcvarsall.bat

:: the path where the 3dtk sources are
set sourcedir=%~1

if "%sourcedir%" == "" (
	echo "Usage: %0 sourcedir outdir [v14x]"
	exit /B 1
)

:: the path where you want the resulting binaries
set outdir=%~2

if "%outdir%" == "" (
	echo "Usage: %0 sourcedir outdir [v14x]"
	exit /B 1
) else ( echo outdir: %outdir% )

:: there is no elsif in windows batch
set toolset=%~3
if "%toolset%" == "" (
	set "toolset=v142"
)
if "%toolset%" == "v141" (
	set "generator=Visual Studio 16 2017"
	set "triplet=x64-windows-v141"
)
if "%toolset%" == "v142" (
	set "generator=Visual Studio 16 2019"
	set "triplet=x64-windows-v142"
)
:: we would like to abort if the third argument is anything other than the
:: empty string or the allowed toolset values, but without elsif and without
:: AND operator, we just give up...

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
) else ( echo sourcedir: %sourcedir% )

if not exist "%outdir%" mkdir "%outdir%"

:: 2020.04 is not recent enough because of
:: https://github.com/microsoft/vcpkg/issues/10642
:: set vcpkgcommit=2020.04
:: set vcpkghash=f8-05-14-11-81-05-67-01-15-89-62-a3-21-fb-a9-91
set vcpkgcommit=218e87ca1c89a510a6a91ed72647219734918080
set vcpkghash=6d-cd-ec-ad-eb-3e-54-d2-08-88-8d-90-70-46-8f-46
set vcpkgurl=https://github.com/Microsoft/vcpkg/archive/!vcpkgcommit!.zip
set vcpkgzip=%outdir%\vcpkg.zip
set vcpkgdir=%outdir%\3rdparty\vcpkg
where vcpkg
:: there is no AND or OR logical operator in windows batch
if %ERRORLEVEL% NEQ 0 (
	if not exist %vcpkgdir% (
		call:reset_error
		if not exist !vcpkgzip! (
			echo downloading !vcpkgzip!...
			call:download !vcpkgurl! !vcpkgzip!
		)
		echo checking md5sum of !vcpkgzip!...
		call:checkmd5 !vcpkgzip! !vcpkghash!
		if !ERRORLEVEL! GEQ 1 (
			echo md5sum mismatch
			exit /B 1
		)
		echo extracting !vcpkgzip! into !vcpkgdir!...
		call:unzip !vcpkgzip! !vcpkgdir!
		if !ERRORLEVEL! GEQ 1 (
			echo vcpkg unzip failed
			exit /B 1
		)
		:: windows does not like long paths
		:: including the git commit hash in the path makes building qt fail
		move !vcpkgdir! "!vcpkgdir!.tmp"
		if !ERRORLEVEL! GEQ 1 (
			echo moving !vcpkgdir! to "!vcpkgdir!.tmp" failed
			exit /B 1
		)
		move "!vcpkgdir!.tmp\vcpkg-!vcpkgcommit!" !vcpkgdir!
		if !ERRORLEVEL! GEQ 1 (
			echo moving "!vcpkgdir!.tmp\vcpkg-!vcpkgcommit!" to !vcpkgdir! failed
			exit /B 1
		)
		rmdir "!vcpkgdir!.tmp"
	)
	set vcpkgexe=!vcpkgdir!\vcpkg.exe
) else (
	:: equivalent of vcpkgexe=$(where vcpkg)
	for /f %%i in ('where vcpkg') do set vcpkgexe=%%i
	:: equivalent of vcpkgdir=$(dirname vcpkgexe)
	for %%F in ("!vcpkgexe!") do set vcpkgdir=%%~dpF

	git -C !vcpkgdir! pull
	git -C !vcpkgdir! reset --hard !vcpkgcommi!
)

echo vcpkgexe: %vcpkgexe%
echo vcpkgdir: %vcpkgdir%

:: patch vpkg so that boost-thread and boost-fiber compile
:: https://github.com/microsoft/vcpkg/issues/7559
:: https://github.com/microsoft/vcpkg/issues/8613
:: https://github.com/microsoft/vcpkg/pull/9224
echo patching vcpkg...
git apply --unsafe-paths --directory=%vcpkgdir% "%sourcedir%\windows\0001-boost-modular-build-helper-Fix-build-issues-when-rel.patch"
echo copy triplets...
copy "%sourcedir%\windows\x64-windows-v141.cmake" "%vcpkgdir%\triplets\x64-windows-v141.cmake"
copy "%sourcedir%\windows\x64-windows-v142.cmake" "%vcpkgdir%\triplets\x64-windows-v142.cmake"
echo building vcpkg...
:: have to use call or otherwise bootstrap-vcpkg.bat will exit everything
call %vcpkgdir%\bootstrap-vcpkg.bat -disableMetrics
if %ERRORLEVEL% GEQ 1 (
	echo bootstrap-vcpkg.bat failed
	exit /B 1
)

set cmakedir=%outdir%\3rdparty\cmake

set cmakezip=%outdir%\cmake-3.12.4-win64-x64.zip
set cmakeurl=https://cmake.org/files/v3.12/cmake-3.12.4-win64-x64.zip
set cmakehash=f4-e8-13-07-8f-51-80-aa-ee-a4-5a-5b-87-5b-16-97

where cmake
:: there is no AND or OR logical operator in windows batch
if %ERRORLEVEL% NEQ 0 (
	if not exist %cmakedir% (
		echo Could not find cmake. Trying to build it from download...
		call:reset_error
		if not exist !cmakezip! (
			echo downloading !cmakezip!...
			call:download !cmakeurl! !cmakezip!
		)
		echo checking md5sum of !cmakezip!...
		call:checkmd5 !cmakezip! !cmakehash!
		if !ERRORLEVEL! GEQ 1 (
			echo md5sum mismatch
			exit /B 1
		)
		echo extracting !cmakezip! into !cmakedir!...
		call:unzip !cmakezip! !cmakedir!
		if !ERRORLEVEL! GEQ 1 (
			echo cmake unzip failed
			exit /B 1
		)
	)
	set cmakeexe=!cmakedir!\cmake-3.12.4-win64-x64\bin\cmake.exe
) else (
	set cmakeexe=cmake
)

echo cmakeexe: %cmakeexe%

%vcpkgexe% update

if %ERRORLEVEL% GEQ 1 (
	echo vcpkg update failed
	exit /B 1
) else ( echo vcpkg update succeeded )

%vcpkgexe% remove --outdated --recurse
if %ERRORLEVEL% GEQ 1 (
	echo vcpkg remove --outdated failed
	exit /B 1
) else ( echo vcpkg remove --outdated succeeded )

%vcpkgexe% upgrade --no-dry-run
if %ERRORLEVEL% GEQ 1 (
	echo vcpkg upgrade failed
	exit /B 1
) else ( echo vcpkg upgrade succeeded )

%vcpkgexe% --triplet %triplet% install ^
	qt5 ^
	libpng ^
	boost ^
	opencv ^
	wxwidgets ^
	eigen3 ^
	python3 ^
	boost-python ^
	zlib ^
	freeglut ^
	cgal ^
	glfw3 ^
	libconfig ^
	libzip ^
	opengl ^
	mpir ^
	pthreads ^
	suitesparse
if %ERRORLEVEL% GEQ 1 (
	echo vcpkg install failed
	exit /B 1
) else ( echo installed all dependencies managed by vcpkg )

%vcpkgexe% remove --outdated
if %ERRORLEVEL% GEQ 1 (
	echo vcpkg remove --outdated failed
	exit /B 1
) else ( echo vcpkg remove --outdated succeeded )

:: The package zlib is compatible with built-in CMake targets:
:: 
::     find_package(ZLIB REQUIRED)
::     target_link_libraries(main PRIVATE ZLIB::ZLIB)
:: 
:: The package libpng is compatible with built-in CMake targets:
:: 
::     find_package(PNG REQUIRED)
::     target_link_libraries(main PRIVATE PNG::PNG)
:: 
:: The package eigen3:x64-windows provides CMake targets:
:: 
::     find_package(Eigen3 CONFIG REQUIRED)
::     target_link_libraries(main PRIVATE Eigen3::Eigen)
:: 
:: The package opencv provides CMake integration:
:: 
::     find_package(OpenCV REQUIRED)
::     target_include_directories(main PRIVATE ${OpenCV_INCLUDE_DIRS})
::     target_link_libraries(main PRIVATE ${OpenCV_LIBS})
:: 
:: The package freeglut is compatible with built-in CMake targets:
:: 
::     find_package(GLUT REQUIRED)
::     target_link_libraries(main PRIVATE GLUT::GLUT)
:: 
:: The package suitesparse:x64-windows provides CMake targets:
:: 
::     find_package(suitesparse-5.1.2 CONFIG REQUIRED)
::     # Note: 8 target(s) were omitted.
::     target_link_libraries(main PRIVATE SuiteSparse::amd SuiteSparse::btf SuiteSparse::klu SuiteSparse::ldl)

echo "cmake: %cmakeexe%"
:: FIXME: starting with CMake 3.13 we can use the (finally documented) -S
:: option instead of the (long undocumented but still working) -H option
:: FIXME: Does currently not work with sourcedir/outdir given by relative paths
"%cmakeexe%" ^
	-H"%sourcedir%" ^
	-B"%outdir%" ^
	-DCMAKE_BUILD_TYPE=RelWithDebInfo ^
	-DCMAKE_TOOLCHAIN_FILE=%vcpkgdir%\scripts\buildsystems\vcpkg.cmake ^
	-DOUTPUT_DIRECTORY:PATH=%outdir% ^
	-DWITH_PYTHON=OFF ^
	-DWITH_LASLIB=OFF ^
	-DWITH_XMLRPC=OFF ^
	-DWITH_WXWIDGETS=OFF ^
	-DWITH_FTGL=OFF ^
	-DWITH_ROS=OFF ^
	-G"%generator%" ^
	-A x64

if %ERRORLEVEL% GEQ 1 (
	echo cmake config failed
	exit /B 1
) else ( echo cmake config done )

"%cmakeexe%" --build "%outdir%" --config %buildtype% -- /m

if %ERRORLEVEL% GEQ 1 (
	echo cmake --build failed
	exit /B 1
) else ( echo cmake --build succeeded )

set CTEST_OUTPUT_ON_FAILURE=true
"%cmakeexe%" --build "%outdir%" --config %buildtype% --target RUN_TESTS

if %ERRORLEVEL% GEQ 1 (
	echo cmake --build --target RUN_TESTS failed
	exit /B 1
)

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
		"Expand-Archive """"%~1"""" -Force -DestinationPath """"%~2"""""
	if %ERRORLEVEL% GEQ 1 (
		exit /B 1
	)
	exit /B 0

:reset_error
	exit /b 0
