#!/bin/sh

#build script for macOS on travis 
echo "travis_fold:start:brew_install"
	#uninstall python numpy because is not compatible with brew opencv
	/usr/bin/yes | pip2 uninstall numpy
	# update brew
	brew update
	# install packageds with brew
	brew bundle
	#export qt path, installed by brew
	export PATH="/usr/local/opt/qt/bin:$PATH"
echo "travis_fold:end:brew_install"

#configure and make 3dtk
echo "travis_fold:start:configure_3dtk"
	mkdir .build
	cmake -H. -B.build -DWITH_PYTHON=OFF -DWITH_OPENMP=OFF -DWITH_FTGL=OFF $CMAKEOPTS -G "Ninja"
echo "travis_fold:end:configure_3dtk"

# build 3dtk
echo "travis_fold:start:build_3dtk"
	cmake --build .build
echo "travis_fold:end:build_3dtk"