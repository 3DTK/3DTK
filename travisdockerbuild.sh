#!/bin/sh

set -eu

if [ "$#" -ne 2 ] && [ "$#" -ne 3 ]; then
	echo "usage: $0 derivative distribution [compiler]" >&2
	exit 1
fi

DERIV=$1
DIST=$2
CC=
if [ "$#" -gt 2 ]; then
	CC=$3
fi

case "$DERIV" in
	debian)
		MIRROR="http://deb.debian.org/debian"
		SECMIRROR="http://security.debian.org/debian-security"
		COMP="main"
		;;
	ubuntu)
		MIRROR="http://archive.ubuntu.com/ubuntu"
		SECMIRROR="http://security.ubuntu.com/ubuntu"
		COMP="main restricted universe multiverse"
		;;
	*)
		echo "unknown derivative: $DERIV" >&2
		exit 1
esac

CMAKEOPTS="-DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_VERBOSE_MAKEFILE=ON"

case "$DIST" in
	buster|bullseye|sid|focal)
		CMAKEOPTS="$CMAKEOPTS -DWITH_ROS=ON"
		# if buster or unstable are run inside Docker, then generating
		# moc_GLWidget.cpp will fail with:
		# standard input:0: Note: No relevant classes found. No output generated.
		# this is because under docker, the statx system call is not
		# supported
		# a workaround is to install an updated libseccomp library
		arch=$(dpkg --print-architecture)
		wget https://launchpad.net/ubuntu/+archive/primary/+files/libseccomp2_2.4.1-0ubuntu0.16.04.2_$arch.deb -O /tmp/libseccomp.deb
		sudo apt-get install /tmp/libseccomp.deb
		;;
	xenial|bionic|stretch)
		# nothing to do
		;;
	*)
		echo "unknown distribution: $DIST" >&2
		exit 1
esac

cat > Dockerfile <<EOF
FROM $DERIV:$DIST
WORKDIR $(pwd)
COPY . .
RUN rm -f Dockerfile
RUN echo "deb $MIRROR $DIST $COMP" > /etc/apt/sources.list
RUN echo force-unsafe-io > /etc/dpkg/dpkg.cfg.d/force-unsafe-io
RUN echo 'Acquire::EnableSrvRecords "false";' > /etc/apt/apt.conf.d/90srvrecords
EOF

case "$DIST" in
	stretch|buster)
		cat >> Dockerfile <<EOF
RUN echo "deb $MIRROR $DIST-updates $COMP" >> /etc/apt/sources.list
RUN echo "deb $SECMIRROR $DIST/updates $COMP" >> /etc/apt/sources.list
EOF
		;;
	xenial|bionic|focal)
		cat >> Dockerfile <<EOF
RUN echo "deb $MIRROR $DIST-updates $COMP" >> /etc/apt/sources.list
RUN echo "deb $SECMIRROR $DIST-security $COMP" >> /etc/apt/sources.list
EOF
		;;
	sid|bullseye)
		# nothing to do
		;;
	*)
		echo "unknown distribution: $DIST" >&2
		exit 1
esac

TAG="3dtk.docker.$DERIV.$DIST"

echo "travis_fold:start:docker_build"

docker build --tag="$TAG" .

echo "travis_fold:end:docker_build"

GENERATOR=Ninja
APT="apt-get install --yes --no-install-recommends -o Debug::pkgProblemResolver=yes"

{
	echo "set -exu";
	echo "export DEBIAN_FRONTEND=noninteractive";
	echo "export DEBCONF_NONINTERACTIVE_SEEN=true";
	echo "echo travis_fold:start:docker_setup";
	echo "dpkg -l";
	echo "cat /etc/apt/sources.list";
	echo "apt-get update";
	echo "apt-get dist-upgrade --yes";
	echo "$APT equivs ninja-build build-essential";
	if [ -z "$CC" ]; then
    case "$DIST" in
	    buster|bullseye|sid|focal)
		    echo "equivs-build doc/equivs/control.$DERIV.$DIST.ros";
		    ;;
      *)
		    echo "equivs-build doc/equivs/control.$DERIV.$DIST";
    esac
	else
		echo "equivs-build doc/equivs/control.$DERIV.$DIST.$CC";
	fi
	echo "$APT ./3dtk-build-deps_1.0_all.deb";
	echo "echo travis_fold:end:docker_setup";
	echo "mkdir .build";
	echo "cmake -H. -B.build $CMAKEOPTS -G \"$GENERATOR\"";
	echo "cmake --build .build --config RelWithDebInfo";
	echo "CTEST_OUTPUT_ON_FAILURE=true cmake --build .build --config RelWithDebInfo --target test";
} | docker run --interactive --rm "$TAG" sh -
