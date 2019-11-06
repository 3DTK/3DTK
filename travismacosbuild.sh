#!/bin/sh

#build script for macOS on travis 
set -eu

echo "travis_fold:start:brew_install"
    brew update || true
	brew update
	#uninstall python numpy because is not compatible with brew opencv
	/usr/bin/yes | pip2 uninstall numpy
	# update brew
	brew tap brewsci/science
	brew update
	#brew upgrade
	# install packageds with brew
	brew bundle
	#export qt path, installed by brew
	export PATH="/usr/local/opt/qt/bin:$PATH"
echo "travis_fold:end:brew_install"

#configure and make 3dtk
echo "travis_fold:start:configure_3dtk"
	mkdir .build
	cmake -H. -B.build -DWITH_PYTHON=OFF -DWITH_OPENMP=OFF -DWITH_FTGL=OFF -DWITH_OPENCV=OFF
echo "travis_fold:end:configure_3dtk"

# build 3dtk
echo "travis_fold:start:build_3dtk"
	cmake --build .build
echo "travis_fold:end:build_3dtk"

# test 3dtk
echo "travis_fold:start:test_3dtk"
	CTEST_OUTPUT_ON_FAILURE=true cmake --build .build --target test || true
	echo .build/testing/show/show_uosr.png
	base64 .build/testing/show/show_uosr.png
	echo .build/testing/show/show_xyzr.png
	base64 .build/testing/show/show_xyzr.png
echo "travis_fold:end:test_3dtk"
