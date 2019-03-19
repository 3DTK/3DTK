# Installation

## Ubuntu 16.04 

The easiest way to make sure that you install the right dependencies for
unpackaged software like _3DTK_ on Debian based distributions is to build a
meta package using the equivs tool. For _3DTK_ we ship the control information
of such a meta package in this repository so that you can use the equivs tool
to build and install a meta package which will then in turn make sure that you
have all the required dependencies on your system. The advantage compared to
installing all dependencies manually is that by removing the meta package
`3dtk-build-deps` you created can also automatically remove all the
dependencies that were installed with it and are not needed anymore.

Run `apt-cache policy` to find out whether you have the universe and multiverse
repositories enabled. If not, run:

```
$ echo deb http://archive.ubuntu.com/ubuntu xenial main universe multiverse | sudo tee -a /etc/apt/sources.list > /dev/null
$ sudo apt-get update
```

To download 3DTK and install its dependencies, run:

```
$ sudo apt-get install --no-install-recommends equivs subversion
$ svn checkout svn://svn.code.sf.net/p/slam6d/code/trunk slam6d-code
$ cd slam6d-code
$ equivs-build doc/equivs/control.ubuntu.xenial
$ sudo dpkg -i 3dtk-build-deps_1.0_all.deb
$ sudo apt-get -f install --no-install-recommends
$ make
```

You can now use 3DTK, for example by running these tools:

```
$ bin/slam6D dat
$ bin/show dat
```

### Building with Ninja

Ninja builds 3DTK faster than GNU Make and will automatically build in
parallel. Follow the steps above, but before running `make`, install Ninja:

```
$ sudo apt-get install ninja-build
```

Then tell our Makefile to use Ninja:

```
$ CMAKE_GENERATOR=Ninja make
```

## Debian Jessie

Read and follow the instructions for Ubuntu 16.04, do *not* add the Ubuntu
universe and multiverse repositories, and make the following adaptions:

```
$ equivs-build doc/equivs/control.debian.jessie
```

Before compiling ($make) disable (switch to OFF) the options:
`WITH_GLFW`, `WITH_QT`, `WITH_CGAL`

## macOS 10.13 High Sierra

The easiest way to install all required dependencies is to use Homebrew. You can install Homebrew as follows:
```
$/usr/bin/ruby -e "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/master/install)"
```
for more information about Homebrew have a look at [Homebrew](http://brew.sh/index_de.html).
After this, the commands to install the dependencies and build are:
```
$ brew update
$ brew bundle
$ export PATH="/usr/local/opt/qt/bin:$PATH"
$ mkdir .build
$ cmake -H. -B.build -DWITH_PYTHON=OFF -DWITH_OPENMP=OFF -DWITH_FTGL=OFF $CMAKEOPTS -G "Ninja"
$ cmake --build .build
```


## Fedora 25
```
$ dnf install make gcc gcc-c++ libzip-devel ann-devel boost-devel opencv-devel mesa-libGL-devel freeglut-devel libXmu-devel libXi-devel suitesparse-devel
```
