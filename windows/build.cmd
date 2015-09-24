:: this script is to build 3dtk on 64bit windows with visual studio 2013
:: if you require support for 32bit windows, please send patches
:: this was tested on Windos 7 64bit

:: To run CL.exe manually from a terminal, you must first setup your
:: environment using a call to
:: C:\Program Files (x86)\Microsoft Visual Studio 12.0\VC\vcvarsall.bat

:: you might want to configure the following variables before you run this
:: script:

:: the path to your CMake executable
set cmakeexe=C:/Program Files (x86)/CMake/bin/cmake.exe

:: the path where the 3dtk sources are
set sourcedir=Z:/slam6d/

:: the path where you want the resulting binaries
set outdir=C:/slam6d/

:: the build type (one of Debug, Release, RelWithDebInfo and MinSizeRel)
set buildtype=Release

:: this script must have the extension .cmd under windows or otherwise (if
:: it's named .bat for example) %ERRORLEVEL% will not be reset but keep being
:: false even if executions of commands succeed
::
:: this script uses embedded powershell to download files and check their md5
:: sums. The script is not written in powershell itself to avoid stupid popups
:: about this script being insecure, do you really want to execute it, yadda,
:: yadda...
::
:: because of powershell, this script needs either Windows 7 or an earlier
:: Windows version with powershell (>= 2.0) installed
::
:: this script hardcodes the visual studio version. This is because MSVC 12.0
:: is the last version with downloadable binaries available for boost and
:: opencv
::
:: the source directory must not be in the root of a drive letter under
:: windows. See:
::  - http://stackoverflow.com/questions/31871972/
::  - http://public.kitware.com/pipermail/cmake/2015-August/061332.html
::  - http://www.cmake.org/Bug/view.php?id=15134
::  - http://www.cmake.org/Bug/view.php?id=10072
::
:: also, *never* change this file while the script is still running. It seems
:: that on windows, the script is not read into memory and executed by the
:: interpreter but instead read on demand.

@echo off

set opencvdir=%sourcedir%/3rdparty/opencv/
set boostdir=%sourcedir%/3rdparty/boost/

set boostexe=boost_1_58_0-msvc-12.0-64.exe
set boosturl=http://netcologne.dl.sourceforge.net/project/boost/boost-binaries/1.58.0/boost_1_58_0-msvc-12.0-64.exe
set boosthash=5f-b8-23-3b-ad-cf-d5-a6-28-12-a2-30-06-35-db-c9

set opencvexe=opencv-2.4.9.exe
set opencvurl=http://freefr.dl.sourceforge.net/project/opencvlibrary/opencv-win/2.4.9/opencv-2.4.9.exe
set opencvhash=cd-c4-be-ed-03-07-e9-02-c3-a5-1f-71-45-bd-5d-c7

if not exist %boostdir% (
	if not exist %boostexe% (
		echo downloading %boostexe%...
		call:download %boosturl% %boostexe%
	)
	echo checking md5sum of %boostexe%...
	call:checkmd5 %boostexe% %boosthash%
	if ERRORLEVEL 1 (
		echo md5sum mismatch
		exit /B 1
	)
	echo extracting %boostexe% into %boostdir%...
	%boostexe% /silent /nocancel /dir="%boostdir%"
	if %ERRORLEVEL% GEQ 1 (
		echo boost install failed
		exit /B 1
	)
)

if not exist %opencvdir% (
	if not exist %opencvexe% (
		echo downloading %opencvexe%...
		call:download %opencvurl% %opencvexe%
	)
	echo checking md5sum of %opencvexe%...
	call:checkmd5 %opencvexe% %opencvhash%
	if ERRORLEVEL 1 (
		echo md5sum mismatch
		exit /B 1
	)
	echo extracting %opencvexe% into %opencvdir%...
	%opencvexe% -o"%opencvdir%" -y
	if %ERRORLEVEL% GEQ 1 (
		echo opencv install failed
		exit /B 1
	)
)

"%cmakeexe%" ^
	-G "Visual Studio 12 2013 Win64" ^
	-DZLIB_LIBRARY:FILEPATH=%sourcedir%/3rdparty/windows/zlib.lib ^
	-DBOOST_LIBRARYDIR:PATH=%boostdir%/lib64-msvc-12.0 ^
	-DBOOST_ROOT:PATH=%boostdir% ^
	-DGLUT_glut_LIBRARY:FILEPATH=%sourcedir%/3rdparty/windows/freeglut/lib/x64/freeglut.lib ^
	-DZLIB_INCLUDE_DIR:PATH=%sourcedir%/3rdparty/windows/zlib ^
	-DGLUT_INCLUDE_DIR:PATH=%sourcedir%/3rdparty/windows/freeglut/include ^
	-DOpenCV_DIR:PATH=%opencvdir%/opencv/build ^
	-DOUTPUT_DIRECTORY:PATH=%outdir% ^
	%sourcedir%

if %ERRORLEVEL% GEQ 1 (
	echo cmake config failed
	exit /B 1
)

"%cmakeexe%" --build . --config %buildtype%

if %ERRORLEVEL% GEQ 1 (
	echo cmake --build failed
	exit /B 1
)

call:copy %sourcedir%/3rdparty/windows/freeglut/bin/x64/freeglut.dll %outdir%/bin/%buildtype%

if %ERRORLEVEL% GEQ 1 (
	echo copy failed
	exit /B 1
)

pause

goto:eof

:: we cannot use the normal "copy" command because that one does not support
:: forward slashes in paths even though the rest of the world (including
:: windows file explorer) does
:copy
	powershell -command ^
		"Copy-Item """"%~1"""" """"%~2"""""
	if %ERRORLEVEL% GEQ 1 (
		exit /B 1
	)
	exit /B 0

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
		"$wc = New-Object System.Net.WebClient;" ^
		"$wc.DownloadFile(""""%~1""", ^
			"""%~2"""")";
	if %ERRORLEVEL% GEQ 1 (
		exit /B 1
	)
	exit /B 0
