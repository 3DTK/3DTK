:: this script is to build 3dtk on 64bit windows with visual studio 2013
:: if you require support for 32bit windows, please send patches
:: this was tested on Windos 7 64bit

:: To run CL.exe manually from a terminal, you must first setup your
:: environment using a call to
:: C:\Program Files (x86)\Microsoft Visual Studio 12.0\VC\vcvarsall.bat

:: you might want to configure the following variables before you run this
:: script:

:: the path where the 3dtk sources are
set sourcedir=Z:/3dtk/

:: the path where you want the resulting binaries
set outdir=C:/slam6d/

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

@echo off

if not exist %sourcedir% (
	echo %sourcedir% does not exist. Make sure the sourcedir variable is set to the path of the 3DTK sources.
	exit /B 1
)


for %%p in (
		%sourcedir%/3rdparty/windows/freeglut/lib/x64/freeglut.lib
		%sourcedir%/3rdparty/windows/freeglut/include
		%sourcedir%/3rdparty/windows/zlib.lib
		%sourcedir%/3rdparty/windows/zlib
	) do (
		if not exist %%p (
			echo %%p does not exist - does %sourcedir% really contain the 3DTK sources?
			exit /B 1
		)
)

if not exist %outdir% (
	echo %outdir% does not exist. Make sure the outdir variable is set to an existing path.
	exit /B 1
)

set opencvdir=%outdir%/3rdparty/opencv/
set boostdir=%outdir%/3rdparty/boost/
set cmakedir=%outdir%/3rdparty/cmake/
set libpngdir=%outdir%/3rdparty/libpng/

set cmakeexe=%outdir%/3rdparty/cmake/cmake-3.7.2-win64-x64/bin/cmake.exe
set cmakezip=%outdir%/cmake-3.7.2-win64-x64.zip
set cmakeurl=https://cmake.org/files/v3.7/cmake-3.7.2-win64-x64.zip
set cmakehash=b5-e9-fa-6c-cb-56-06-66-84-19-2f-4f-1d-30-54-93

set boostexe=%outdir%/boost_1_63_0-msvc-14.0-64.exe
set boosturl=https://downloads.sourceforge.net/project/boost/boost-binaries/1.63.0/boost_1_63_0-msvc-14.0-64.exe
set boosthash=e6-87-21-e0-da-79-18-df-0c-d7-86-e9-f2-25-98-39

set opencvexe=%outdir%/opencv-3.2.0-vc14.exe
set opencvurl=https://downloads.sourceforge.net/project/opencvlibrary/opencv-win/3.2.0/opencv-3.2.0-vc14.exe
set opencvhash=76-31-e7-08-a9-ae-03-65-69-e4-00-ba-43-88-68-61

set libpngexe=%outdir%/libpng-1.2.37-setup.exe
set libpngurl=https://downloads.sourceforge.net/project/gnuwin32/libpng/1.2.37/libpng-1.2.37-setup.exe
set libpnghash=f9-d1-c1-54-d4-94-a8-7e-7d-02-03-0d-f9-d6-e5-01

if not exist %boostdir% (
	if not exist %boostexe% (
		echo downloading %boosturl% to %boostexe%...
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
		echo downloading %opencvurl% to %opencvexe%...
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

if not exist %libpngdir% (
	if not exist %libpngexe% (
		echo downloading %libpngexe%...
		call:download %libpngurl% %libpngexe%
	)
	echo checking md5sum of %libpngexe%...
	call:checkmd5 %libpngexe% %libpnghash%
	if ERRORLEVEL 1 (
		echo md5sum mismatch
		exit /B 1
	)
	echo extracting %libpngexe% into %libpngdir%...
    %libpngexe% /DIR="%libpngdir%" /VERYSILENT /SUPPRESSMSGBOXES /NORESTART /SP-
	if %ERRORLEVEL% GEQ 1 (
		echo cmake unzip failed
		exit /B 1
	)
)

:: use setlocal to make sure that the directory is only changed for this part
:: of the script and not on the outside
setlocal
:: need /d if %outdir% is a different drive letter than the current working
:: directory
cd /d %outdir%
"%cmakeexe%" ^
	-G "Visual Studio 14 2015 Win64" ^
	-DZLIB_LIBRARY:FILEPATH=%sourcedir%/3rdparty/windows/zlib.lib ^
	-DBOOST_LIBRARYDIR:PATH=%boostdir%/lib64-msvc-12.0 ^
	-DBOOST_ROOT:PATH=%boostdir% ^
	-DGLUT_glut_LIBRARY:FILEPATH=%sourcedir%/3rdparty/windows/freeglut/lib/x64/freeglut.lib ^
	-DZLIB_INCLUDE_DIR:PATH=%sourcedir%/3rdparty/windows/zlib ^
	-DPNG_PNG_INCLUDE_DIR:PATH=%libpngdir%/include ^
	-DPNG_LIBRARY:FILEPATH=%libpngdir%/lib/libpng.lib ^
	-DGLUT_INCLUDE_DIR:PATH=%sourcedir%/3rdparty/windows/freeglut/include ^
	-DOpenCV_DIR:PATH=%opencvdir%/opencv/build ^
	-DOUTPUT_DIRECTORY:PATH=%outdir% ^
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

:unzip
	powershell -command ^
		"Expand-Archive """"%~1"""" -DestinationPath """"%~2"""""
	if %ERRORLEVEL% GEQ 1 (
		exit /B 1
	)
	exit /B 0

