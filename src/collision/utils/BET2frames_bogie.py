#!/usr/bin/env python

import sys
import itertools
from math import pi, sin, cos, sqrt, atan2
import numpy as np

# this function receives a line segment defined by two points C and D,
# one fix point A of the first bogie and the length b to the second bogie
#
# it calculates whether the second bogie falls on the line segment
# and if yes, returns the point where it intersects the line segment
#
# we end up with the following system of equations:
# A + B = C + a(D-C)
# b^2 = Bx^2 + By^2 + Bz^2
#
# where a is the length of the vector from C to D
# we solve for a and check whether the vector points to a position between
# C and D in which case it is between 0.0 and 1.0
def intersects_segment(C, D, A, b):
    cx,cy,cz = C
    dx,dy,dz = D
    ax,ay,az = A
    ex,ey,ez = dx-cx,dy-cy,dz-cz
    fx,fy,fz = ax-cx,ay-cy,az-cz
    j,k,l = ex**2, ey**2, ez**2
    m,n,o = fx**2, fy**2, fz**2
    p,q,r = ex*fx, ey*fy, ez*fz
    g = p+q+r
    h = j+k+l
    if h == 0:
        return None
    i = h*b**2 + 2*(p*r + q*r + p*q) - m*(k + l) - n*(j + l) - o*(j + k)
    # quadratic formula, so two solutions:
    a1 = g/h + sqrt(i)/h
    if 0 <= a1 <= 1.0:
        return (cx+a1*ex, cy+a1*ey, cz+a1*ez)
    a2 = g/h - sqrt(i)/h
    if 0 <= a2 <= 1.0:
        return (cx+a2*ex, cy+a2*ey, cz+a2*ez)
    return None

def pairwise(iterable):
    "s -> (s0,s1), (s1,s2), (s2, s3), ..."
    a, b = itertools.tee(iterable)
    next(b, None)
    return itertools.izip(a, b)

if len(sys.argv) not in [5,6]:
    print "usage: %s lon lat height bogiedist [file.bet]"
    exit(1)

lons = float(sys.argv[1])
lats = float(sys.argv[2])
height = float(sys.argv[3])
bogiedist = float(sys.argv[4])

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

lengths = list()
# calculate list of line segment lengths
for (x1,y1,z1),(x2,y2,z2) in pairwise(values):
    a = x2-x1
    b = y2-y1
    c = z2-z1
    lengths.append(sqrt(a**2+b**2+c**2))

rx = lambda y: np.matrix([[      1,      0,      0],
                          [      0, cos(y),-sin(y)],
                          [      0, sin(y), cos(y)]])
ry = lambda y: np.matrix([[ cos(y),      0, sin(y)],
                          [      0,      1,      0],
                          [-sin(y),      0, cos(y)]])
rz = lambda y: np.matrix([[ cos(y),-sin(y),      0],
                          [ sin(y), cos(y),      0],
                          [     0,       0,      1]])

# re-calculate yaw, pitch, roll
# assume zero roll
for i,(lon,lat,h) in enumerate(values):
    # find the point of the trajectory for the other bogie
    # only need to start searching after the sum of segments is greater than
    # the bogie length
    ssum = 0
    for j,seglen in enumerate(lengths[i:]):
        ssum += seglen
        if ssum > bogiedist:
            break
    inters = None
    for (x1,y1,z1),(x2,y2,z2) in pairwise(values[i+j:]):
        inters = intersects_segment((x1,y1,z1),(x2,y2,z2),(lon,lat,h),bogiedist)
        if inters:
            break
    if not inters:
        break
    bx,by,bz = inters
    dx,dy,dz = bx-lon,by-lat,bz-h
    yaw = -atan2(dx,dy)
    pitch = -atan2(dz,dy)
    # recalculate the position of the center of the wagon
    newlon,newlat,newh = lon+0.5*dx, lat+0.5*dy, h+0.5*dz
    r = ry(0)*rx(pitch)*rz(yaw)
    f = list(r.flat)
    inMatrix = f[0:3]+[newlon]+f[3:6]+[newlat]+f[6:9]+[newh]+[0,0,0,1]
    # convert rotation matrix to 3dtk format (taken from riegl2frames)
    tMatrix = [0]*16
    tMatrix[0] = inMatrix[5];
    tMatrix[1] = -inMatrix[9];
    tMatrix[2] = -inMatrix[1];
    tMatrix[3] = -inMatrix[13];
    tMatrix[4] = -inMatrix[6];
    tMatrix[5] = inMatrix[10];
    tMatrix[6] = inMatrix[2];
    tMatrix[7] = inMatrix[14];
    tMatrix[8] = -inMatrix[4];
    tMatrix[9] = inMatrix[8];
    tMatrix[10] = inMatrix[0];
    tMatrix[11] = inMatrix[12];
    tMatrix[12] = -inMatrix[7];
    tMatrix[13] = inMatrix[11];
    tMatrix[14] = inMatrix[3];
    tMatrix[15] = inMatrix[15];
    print ' '.join(["%f"%n for n in tMatrix]),
    print " 2"
