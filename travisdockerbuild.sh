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
		CMAKEOPTS="$CMAKEOPTS -DWITH_CGAL=OFF -DWITH_GLFW=OFF -DWITH_QT=OFF -DWITH_PYTHON=OFF -DWITH_LIBZIP=OFF"
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

docker run --interactive --rm "$TAG" sh - <<EOF
set -exu
echo "travis_fold:start:docker_setup"
dpkg -l
cat /etc/apt/sources.list
apt-get update
apt-get dist-upgrade --yes
apt-get install --yes --no-install-recommends -o Debug::pkgProblemResolver=yes equivs ninja-build
equivs-build doc/equivs/control.$DERIV.$DIST
apt-get install --no-install-recommends -o Debug::pkgProblemResolver=yes ././././3dtk-build-deps_1.0_all.deb || true
dpkg --install --force-depends ./3dtk-build-deps_1.0_all.deb
apt-get install --yes --no-install-recommends --fix-broken -o Debug::pkgProblemResolver=yes
echo "travis_fold:end:docker_setup"
mkdir .build
cmake -H. -B.build $CMAKEOPTS -G "Ninja"
cmake --build .build
CTEST_OUTPUT_ON_FAILURE=true cmake --build .build --target test
EOF
