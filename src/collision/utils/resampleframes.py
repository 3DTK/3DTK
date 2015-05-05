#!/usr/bin/env python

from scipy import interpolate
import numpy as np
import sys
import itertools
from math import pi, cos, sqrt

def pairwise(iterable):
    "s -> (s0,s1), (s1,s2), (s2, s3), ..."
    a, b = itertools.tee(iterable)
    next(b, None)
    return itertools.izip(a, b)

if len(sys.argv) not in [2,3]:
    print "usage: %s segsize [file.frames]"
    exit(1)

segsize = float(sys.argv[1])

values = list()

if len(sys.argv) == 3:
    fd = open(sys.argv[2])
else:
    fd = sys.stdin

for line in fd:
    if line.startswith('#'):
        continue
    x,y,z = line.split()[12:15]
    values.append((float(x),float(y),float(z)))

fd.close()

# when creating a spline there must be no duplicate values
seen_values = set()
new_values = list()
for v in values:
    if v in seen_values:
        continue
    new_values.append(v)
    seen_values.add(v)
values = new_values

length = 0
for (x1,y1,z1),(x2,y2,z2) in pairwise(values):
    a = x2-x1
    b = y2-y1
    c = z2-z1
    length += sqrt(a**2+b**2+c**2)

poses_x = [v[0] for v in values]
poses_y = [v[1] for v in values]
poses_z = [v[2] for v in values]

smoothing = 10
subdiv = int(length/segsize)+1

tck,u = interpolate.splprep([poses_x,poses_y,poses_z],s=smoothing)
unew = np.linspace(0,1.0,subdiv)
out = interpolate.splev(unew,tck)

print >>sys.stderr, "overall length: %f"%length

for x,y,z in zip(*out):
    print "1 0 0 0 0 1 0 0 0 0 1 0 %f %f %f 1 3"%(x,y,z)
