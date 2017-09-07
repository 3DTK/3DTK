#!/usr/bin/python3

import argparse
import sys
import ctypes.util
import ctypes
import os
import textwrap

sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..', 'lib'))
try:
    import py3dtk
except ImportError:
    print("Cannot find py3dtk module. Try recompiling 3dtk with WITH_PYTHON set to ON", file=sys.stderr)
    exit(1)

def formatname_to_io_type(string):
    if string.lower() == "uos":
        return py3dtk.IOType.UOS
    if string.lower() == "uosr":
        return py3dtk.IOType.UOSR
    if string.lower() == "xyz":
        return py3dtk.IOType.XYZ
    if string.lower() == "xyzr":
        return py3dtk.IOType.XYZR
    if string.lower() == "riegl_txt":
        return py3dtk.IOType.RIEGL_TXT
    if string.lower() == "rxp":
        return py3dtk.IOType.RXP

def main():
    parser = argparse.ArgumentParser(
            formatter_class=argparse.RawDescriptionHelpFormatter,
            description=textwrap.fill("Find additions of the second scan to the first by finding points in the second scan with at most MAXNUM neighbors within a radius of DIST in the first. By passing only a single scan, this can also be used to clean a pointcloud from random noise."),
            epilog="""Find additions:

    $ %s -s 0 -e 1 --dist=10 --maxnum=10 > additions

Clean scan of noise:

    $ %s -s 0 -e 0 --dist=10 --maxnum=10 --invert > clean
"""%(sys.argv[0], sys.argv[0])
            )
    # FIXME: this application is a prime example of why it would be great if
    # scanio would support passing individual scan files instead of a directory
    #
    # furthermore, this application would benefit from a mechanism that would
    # allow programs to communicate point clouds via UNIX pipes
    parser.add_argument("-s", "--start", type=int, default=0, metavar="NUM")
    parser.add_argument("-e", "--end", type=int, default=-1, metavar="NUM")
    parser.add_argument("-f", "--format", type=formatname_to_io_type, default=py3dtk.IOType.UOS)
    parser.add_argument("--dist", type=float, metavar="DIST", default=1.0)
    parser.add_argument("--maxnum", type=int, metavar="MAXNUM", default=0)
    parser.add_argument("--invert", action="store_true")
    parser.add_argument("directory")
    args = parser.parse_args()

    scanserver = False
    py3dtk.openDirectory(scanserver, args.directory, args.format, args.start, args.end)

    if len(py3dtk.allScans) not in [1,2]:
        print("require either one or two scans, got %d"%len(py3dtk.allScans), file=sys.stderr)
        exit(1)

    scan1 = py3dtk.allScans[0]
    scan1.transformAll(scan1.get_transMatOrg())
    points1 = list(py3dtk.DataXYZ(scan1.get("xyz")))
    if len(py3dtk.allScans) == 1:
        scan2 = scan1
        points2 = points1
    else:
        scan2 = py3dtk.allScans[1]
        scan2.transformAll(scan2.get_transMatOrg())
        points2 = list(py3dtk.DataXYZ(scan2.get("xyz")))
    kdtree1 = py3dtk.KDtree(points1)
    dist2 = args.dist**2
    libc = ctypes.CDLL(ctypes.util.find_library("c"))
    buflen = 100
    buf = ctypes.create_string_buffer(buflen)
    def write_point(f, p):
        ret = libc.snprintf(buf, buflen, b"%a %a %a\n",
            ctypes.c_double(p[0]),
            ctypes.c_double(p[1]),
            ctypes.c_double(p[2]))
        if ret < 0 or ret >= buflen:
            raise Exception("sprintf failed")
        f.write(buf.value)
    for i,p in enumerate(points2):
        print("%f"%((i+1)*100/len(points2)), end="\r", file=sys.stderr)
        pts = kdtree1.kNearestRangeSearch(p, args.maxnum+1, dist2)
        if args.invert != (len(pts) <= args.maxnum):
            write_point(sys.stdout.buffer, p)

if __name__ == "__main__":
    main()
