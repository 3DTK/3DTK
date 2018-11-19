#!/bin/sh

set -eu

if [ "$#" -ne 2 ]; then
	echo "usage: $0 derivative distribution" >&2
	exit 1
fi

DERIV=$1
DIST=$2

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

CMAKEOPTS="-DCMAKE_VERBOSE_MAKEFILE=ON"

case "$DIST" in
	jessie)
		CMAKEOPTS="$CMAKEOPTS -DWITH_GLFW=OFF -DWITH_QT=OFF"
		;;
	trusty)
		CMAKEOPTS="$CMAKEOPTS -DWITH_CGAL=OFF -DWITH_GLFW=OFF -DWITH_QT=OFF -DWITH_PYTHON=OFF -DWITH_LIBZIP=OFF -DWITH_MESH=OFF"
		;;
	sid)
		CMAKEOPTS="$CMAKEOPTS -DWITH_ROS=ON"
		;;
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
	jessie|stretch|buster)
		cat >> Dockerfile <<EOF
RUN echo "deb $MIRROR $DIST-updates $COMP" >> /etc/apt/sources.list
RUN echo "deb $SECMIRROR $DIST/updates $COMP" >> /etc/apt/sources.list
EOF
		;;
	precise|trusty|xenial)
		cat >> Dockerfile <<EOF
RUN echo "deb $MIRROR $DIST-updates $COMP" >> /etc/apt/sources.list
RUN echo "deb $SECMIRROR $DIST-security $COMP" >> /etc/apt/sources.list
EOF
		;;
esac

TAG="3dtk.docker.$DERIV.$DIST"

echo "travis_fold:start:docker_build"

docker build --tag="$TAG" .

echo "travis_fold:end:docker_build"

GENERATOR=Ninja
if [ "$DIST" = "trusty" ]; then
	GENERATOR="Unix Makefiles"
fi
APT="apt-get install --yes --no-install-recommends -o Debug::pkgProblemResolver=yes"

{
	echo "set -exu";
	echo "echo travis_fold:start:docker_setup";
	echo "dpkg -l";
	echo "cat /etc/apt/sources.list";
	echo "apt-get update";
	echo "apt-get dist-upgrade --yes";
	echo "$APT equivs";
	if [ "$DIST" != "trusty" ]; then
		echo "$APT ninja-build";
	fi
	echo "equivs-build doc/equivs/control.$DERIV.$DIST";
	if [ "$DIST" = "trusty" ]; then
		echo "dpkg --install --force-depends ./3dtk-build-deps_1.0_all.deb";
		echo "$APT --fix-broken";
	else
		echo "$APT ./3dtk-build-deps_1.0_all.deb";
	fi
	echo "echo travis_fold:end:docker_setup";
	echo "mkdir .build";
	echo "cmake -H. -B.build $CMAKEOPTS -G \"$GENERATOR\"";
	echo "cmake --build .build";
	echo "CTEST_OUTPUT_ON_FAILURE=true cmake --build .build --target test";
} | docker run --interactive --rm "$TAG" sh -
