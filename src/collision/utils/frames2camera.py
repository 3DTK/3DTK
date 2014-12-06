#!/usr/bin/env python
from itertools import tee, izip
from math import sqrt

def pairwise(iterable):
    a, b = tee(iterable)
    #for i in range(100):
    #    next(b, None)
    next(b, None)
    return izip(a, b)

import sys

poses = list()

for line in sys.stdin:
    if line.startswith('#'):
        continue
    x,y,z = line.split()[12:15]
    poses.append((float(x),float(y),-float(z)))

outdata = "0\n0\n0\n0\n0\n0\n0\n0\n0\n"
outdata += "0\n0\n0\n0\n0\n0\n0\n0\n0\n"
lines = 1
for pos1,pos2 in pairwise(poses):
    length = sqrt((pos1[0]-pos2[0])*(pos1[0]-pos2[0])+(pos1[1]-pos2[1])*(pos1[1]-pos2[1])+(pos1[2]-pos2[2])*(pos1[2]-pos2[2]))
    if length == 0.0:
        continue
    lines += 1
    back = ((pos1[0]-pos2[0])/length,(pos1[1]-pos2[1])/length,(pos1[2]-pos2[2])/length)
    outdata += "%s\n%s\n%s\n"%(pos1[0]+800*back[0],pos1[1]+800*back[1]+400.0,pos1[2]+800*back[2])
    outdata += "%s\n%s\n%s\n"%pos1
    outdata += "%s\n%s\n%s\n"%(pos1[0],pos1[1]+1,pos1[2])

print lines
print outdata,
