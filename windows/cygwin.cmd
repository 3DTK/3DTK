@echo on

set PATH=C:cygwin64\bin
set CYGWIN_ROOT=C:\cygwin64

:: It doesn't make sense to run setup.exe from within cygwin because even
:: though it works, cygwin has to be restarted after upgrading anyways
%CYGWIN_ROOT%\setup-x86_64.exe ^
--no-desktop ^
--no-shortcuts ^
--no-startmenu ^
--quiet-mode ^
--upgrade-also ^
--root %CYGWIN_ROOT% ^
--packages ^
cmake,^
eigen3,^
libGLEW-devel,^
libQt5Core-devel,^
libQt5Gui-devel,^
libXi-devel,^
libXmu-devel,^
libamd-devel,^
libboost-devel,^
libboost_python3-devel,^
libcamd-devel,^
libccolamd-devel,^
libcholmod-devel,^
libcolamd-devel,^
libcxsparse-devel,^
libglut-devel,^
liblapack-devel,^
libopencv-devel,^
libpng-devel,^
libspqr-devel,^
libsuitesparseconfig-devel,^
libwx_baseu3.0-devel,^
libwx_gtk3u3.0-devel,^
libzip-devel,^
python3-devel,^
xorg-server-extra,^
zlib-devel
if %ERRORLEVEL% GEQ 1 (
	echo setup.exe failed
	exit /B 1
)

(
@echo.set -exu
@echo.sourcedir="$(cygpath "${APPVEYOR_BUILD_FOLDER}")"
@echo.mkdir "$sourcedir/build"
@echo.cmake -H"$sourcedir" -B"$sourcedir/build" \
@echo.	-DCMAKE_VERBOSE_MAKEFILE=ON \
@echo.	-DWITH_GLFW=OFF \
@echo.	-DWITH_CGAL=OFF \
@echo.	-DWITH_PYTHON=OFF \
@echo.	-DWITH_WXWIDGETS=OFF
@echo.cmake --build "$sourcedir/build"
@echo.CTEST_OUTPUT_ON_FAILURE=true cmake --build "$sourcedir/build" --target test
) ^
	 | %CYGWIN_ROOT%\bin\sed "s/ \r$//" ^
	 | %CYGWIN_ROOT%\bin\bash -l
if %ERRORLEVEL% GEQ 1 (
	echo build failed
	exit /B 1
)
