@echo on

:: reduce time required to install packages by disabling pacman's disk space checking
C:\msys64\usr\bin\sed -i "s/^CheckSpace/#CheckSpace/g" C:\msys64\etc\pacman.conf

:: Update core packages
call:pacman -Syyuu
if %ERRORLEVEL% GEQ 1 (
	echo pacman -Syyuu failed
	exit /B 1
)

:: Update non-core packages
call:pacman -Suu
if %ERRORLEVEL% GEQ 1 (
	echo pacman -Suu failed
	exit /B 1
)

:: Install required MSYS2 packages
call:pacman -S --needed base-devel
if %ERRORLEVEL% GEQ 1 (
	echo pacman -S --needed base-devel failed
	exit /B 1
)

:: set MINGW_PACKAGE_PREFIX=mingw-w64-x86_64
:: set MSYSTEM=MINGW64
set MINGW_PACKAGE_PREFIX=mingw-w64-i686
set MSYSTEM=MINGW32

echo MINGW_PACKAGE_PREFIX: %MINGW_PACKAGE_PREFIX%
echo APPVEYOR_BUILD_FOLDER: %APPVEYOR_BUILD_FOLDER%

call:pacman -S --needed ^
	%MINGW_PACKAGE_PREFIX%-boost ^
	%MINGW_PACKAGE_PREFIX%-cgal ^
	%MINGW_PACKAGE_PREFIX%-cmake ^
	%MINGW_PACKAGE_PREFIX%-eigen3 ^
	%MINGW_PACKAGE_PREFIX%-freeglut ^
	%MINGW_PACKAGE_PREFIX%-glew ^
	%MINGW_PACKAGE_PREFIX%-glfw ^
	%MINGW_PACKAGE_PREFIX%-libconfig ^
	%MINGW_PACKAGE_PREFIX%-libpng ^
	%MINGW_PACKAGE_PREFIX%-libzip ^
	%MINGW_PACKAGE_PREFIX%-opencv ^
	%MINGW_PACKAGE_PREFIX%-qt5 ^
	%MINGW_PACKAGE_PREFIX%-suitesparse ^
	%MINGW_PACKAGE_PREFIX%-toolchain ^
	%MINGW_PACKAGE_PREFIX%-wxWidgets
if %ERRORLEVEL% GEQ 1 (
	echo pacman package installation failed
	exit /B 1
)

:: Delete unused packages to reduce space used in the Appveyor cache
call:pacman -Sc
if %ERRORLEVEL% GEQ 1 (
	echo pacman -Sc failed
	exit /B 1
)

:: Workaround for CMake not wanting sh.exe on PATH for MinGW
set PATH=%PATH:C:\Program Files\Git\usr\bin;=%
:: set PATH=C:\MinGW\bin;%PATH%
:: set PATH=C:\msys64\mingw64\x86_64-w64-mingw32\bin;%PATH%
:: set PATH=C:\msys64\mingw64\bin;%PATH%
set PATH=C:\msys64\mingw32\bin;%PATH%

echo PATH: %PATH%

md %APPVEYOR_BUILD_FOLDER%\build
cmake -H%APPVEYOR_BUILD_FOLDER% -B%APPVEYOR_BUILD_FOLDER%\build ^
	-DCMAKE_VERBOSE_MAKEFILE=ON ^
	-DWITH_LIBCONFIG=OFF ^
	-DWITH_CGAL=OFF ^
	-DWITH_PYTHON=OFF ^
	-DWITH_LASLIB=OFF ^
	-DWITH_WXWIDGETS=OFF ^
	-G "MinGW Makefiles"
if %ERRORLEVEL% GEQ 1 (
	echo cmake configuration failed
	exit /B 1
)

cmake --build %APPVEYOR_BUILD_FOLDER%\build
if %ERRORLEVEL% GEQ 1 (
	echo cmake --build failed
	exit /B 1
)

set CTEST_OUTPUT_ON_FAILURE=true
cmake --build %APPVEYOR_BUILD_FOLDER%\build --target test
if %ERRORLEVEL% GEQ 1 (
	echo cmake --build --target test failed
	exit /B 1
)

echo "build successful!"

goto:eof

:: run pacman in bash wrapper so that installation scripts can find all
:: necessary binaries
:pacman
	C:\msys64\usr\bin\bash -lc "pacman --noconfirm %*"
	if %ERRORLEVEL% GEQ 1 (
		exit /B 1
	)
	exit /B 0
