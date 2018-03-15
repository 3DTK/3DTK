#!/bin/sh

#build script for macOS on travis 

#export qt path, installed by brew
export PATH="/usr/local/opt/qt/bin:$PATH"

#configure and make 3dtk
mkdir .build
cmake -H. -B.build -DWITH_PYTHON=OFF -DWITH_OPENMP=OFF -DWITH_FTGL=OFF
make