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

if len(sys.argv) not in [5,6]:
    print "usage: %s lon lat height segsize [file.bet]"
    exit(1)

lons = float(sys.argv[1])
lats = float(sys.argv[2])
height = float(sys.argv[3])
segsize = float(sys.argv[4])

r_earth = 6378137
m_lon = (2*pi/360) * r_earth * cos(lats*pi/180)
m_lat = (2*pi/360) * r_earth

values = list()

if len(sys.argv) == 6:
    fd = open(sys.argv[5])
else:
    fd = sys.stdin

for line in fd:
    t,lon,lat,h,r,p,y = line.split()[:7]
    try:
        values.append((
            #float(t),
            m_lon*(float(lon)/100000-lons),
            m_lat*(float(lat)/100000-lats),
            float(h)-height,
            #float(r)*pi/180,
            #float(p)*pi/180,
            #float(y)*pi/180
            ))
    except:
        pass

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

for x,y,z in zip(*out):
    print "0 %f %f %f 0 0 0"%(
            ((x/m_lon)+lons)*100000,
            ((y/m_lat)+lats)*100000,
            z+height
            )
