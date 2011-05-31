#!/bin/sh

# by Pablo d'Angelo <pablo.dangelo@web.de>

set -e

usage()
{
	echo "usage: autopano-complete.sh [options] -o panoproject.pto image1 image2 [...]"
	echo
	echo "options can be:"
	echo "  -o | --output   name    filename of created panorama project"
	echo "  -s | --size     number  downsize images until width and height is"
	echo "                          smaller than number, default 700"
	echo "  -p | --points   number  number of generated control points between,"
	echo "                          each pair, default: 10"
	echo "  -n | --noransac         no ransac detection, useful for fisheye images"
	echo "  -c | --clean            do not reuse keypoints detected in earlier runs,"
	echo "                          deletes old keypoint files."
	exit 1
}

NARG=$#
if [ $NARG -lt 2 ]; then
	usage
	exit 1
fi


os=`uname`
echo "uname: $os"

if [ $os = "Linux" ] ; then
    args=`getopt -o o:s:p:nch -l output:,size:,points:,noransac,clean,help \
         -n "$0" -- "$@"`
    # Note the quotes around `$TEMP': they are essential!
    eval set -- "$args"
elif [ $os = "WindowsNT" ]; then
    echo "On windows"
    # do simplified command line parsing, getopt is not available.
    # all non options arguments need to be specified after the
    # -- option.
else
    # Be conservative and accept only one-letter options.
    # This works fine at least on FreeBSD.
    args=`getopt o:s:p:nch $*`
    set -- $args
fi

if [ $? != 0 ] ; then echo "Terminating..." >&2 ; exit 1 ; fi


POINTS=10;
RANSAC=1;
CLEAN=0;
SIZE=800;

while true ; do
        case "$1" in
                -o|--output)   PANOFILE=$2; shift 2;;
                -s|--size)     SIZE=$2;     shift 2;;
                -p|--points)   POINTS=$2;   shift 2;;
                -n|--noransac) RANSAC=0;    shift 1;;
                -c|--clean)    CLEAN=1;     shift 1;;
                -h|--help)     usage;       shift 1;;
                --)                         shift ; break ;;
                *) echo "Command line parsing error at: $1" ; exit 1 ;;
        esac
done

echo "Remaining arguments ($#):"
for arg do echo '--> '"\`$arg'" ; done

# Allow user to override temporary directory.
TMP=${TMPDIR:-/tmp}

KEYFILES=""
for arg do
        FILENAME=$(basename "$arg").key.gz
	KEYFILES="$KEYFILES $FILENAME"
	if [ -f $FILENAME ]; then
		if [ $CLEAN -ne 0 ]; then
			generatekeys "$arg" $FILENAME $SIZE
		else
			echo "Using previously generated keypoint file: $FILENAME"
		fi
	else
		generatekeys "$arg" $FILENAME $SIZE
	fi
done

echo "keyfiles: $KEYFILES"
ARG="--ransac $RANSAC --maxmatches $POINTS";
autopano $ARG $PANOFILE $KEYFILES

