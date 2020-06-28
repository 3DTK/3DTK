#!/usr/bin/env python3
from itertools import tee
from math import sqrt
import numpy
from scipy import interpolate
import sys

segsize = float(sys.argv[1])

def pairwise(iterable):
    a, b = tee(iterable)
    next(b, None)
    return zip(a, b)


poses = list()
seen = set()

for line in sys.stdin:
    if line.startswith('#'):
        continue
    x,y,z = line.split()[12:15]
    val = (float(x),float(y),-float(z))
    # when creating a spline there must be no duplicate values
    if val in seen:
        continue
    poses.append(val)
    seen.add(val)

length = 0
for (x1,y1,z1),(x2,y2,z2) in pairwise(poses):
    a = x2-x1
    b = y2-y1
    c = z2-z1
    length += sqrt(a**2+b**2+c**2)

if length < segsize:
    print("length less then segsize: %f"%length)
    exit(1)

poses_x, poses_y, poses_z = zip(*poses)

smoothing = 10
subdiv = int(length/segsize)+1

tck,u = interpolate.splprep([poses_x,poses_y,poses_z],s=smoothing)
unew = numpy.linspace(0,1.0,subdiv)
out = interpolate.splev(unew,tck)

outdata = ""
lines = 0
for pos1,pos2 in pairwise(zip(*out)):
    length = sqrt((pos1[0]-pos2[0])*(pos1[0]-pos2[0])+(pos1[1]-pos2[1])*(pos1[1]-pos2[1])+(pos1[2]-pos2[2])*(pos1[2]-pos2[2]))
    if length == 0.0:
        continue
    lines += 1
    back = ((pos1[0]-pos2[0])/length,(pos1[1]-pos2[1])/length,(pos1[2]-pos2[2])/length)
    # FIXME: make the factor configurable
    outdata += "%s\n%s\n%s\n"%(pos1[0]+400*back[0],pos1[1]+400*back[1]+40,pos1[2]+400*back[2])
    outdata += "%s\n%s\n%s\n"%pos1
    outdata += "%s\n%s\n%s\n"%(pos1[0],pos1[1]+1,pos1[2])

print(lines)
print(outdata, end='')
