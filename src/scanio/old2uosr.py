#!/usr/bin/python3
#
# Converts data in the old Kurt 3D format to uosr format
#
# Input data should look like this:
#
# basedir
# ├── 001
# │   ├── position.dat
# │   ├── scan1.dat
# │   ├── scan2.dat
# │   ├── scan3.dat
# │   ├── [...]
# │   └── scanN.dat
# ├── 002
# │   ├── position.dat
# │   ├── scan1.dat
# │   ├── scan2.dat
# │   ├── scan3.dat
# │   ├── [...]
# │   └── scanN.dat
# ├── 002
# ├── [...]
# └── N
#
# The data is written into a directory inside basedir called "uosr".
#
# Usage:
#
#     ./old2uosr.py basedir
#

import itertools
import sys
import os
import math

basedir=sys.argv[1]

outpath=os.path.join(basedir, "uosr")
try:
    os.mkdir(outpath)
except FileExistsError:
    pass

print("Writing to %s" % outpath)
for i in itertools.count(start=1):
    scanpath = os.path.join(basedir, "%03d" % i)
    if not os.path.isdir(scanpath):
        break
    print("Converting %s..." % scanpath)
    pose_in = os.path.join(scanpath, "position.dat")
    pose_out = os.path.join(outpath, "scan%03d.pose" % i)
    with open(pose_in) as f_in, open(pose_out, "w") as f_out:
        euler = [float(n) for n in f_in.read().split()[:6]]
        euler = [p*0.1 for p in euler[:3]] + [p*0.01 for p in euler[3:]]
        f_out.write("%f %f %f\n%f %f %f\n" % tuple(euler))
    scan_out = os.path.join(outpath, "scan%03d.3d" % i)
    with open(scan_out, "w") as f_out:
        for j in itertools.count(start=1):
            scan_in = os.path.join(scanpath, "scan%d.dat" % j)
            if not os.path.isfile(scan_in):
                break
            with open(scan_in) as f_in:
                fstline = f_in.readline().split()
                degree = float(fstline[8])
                cos_angle = math.cos(math.radians(degree))
                sin_angle = math.sin(math.radians(degree))
                for line in f_in:
                    X, Z, _, I = [float(v) for v in line.split()]
                    f_out.write("%f %f %f %f\n" % (X, Z*sin_angle, Z*cos_angle, I))
