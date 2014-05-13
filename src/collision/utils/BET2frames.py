#!/usr/bin/env python

import sys
from math import sin, cos, pi

if len(sys.argv) not in [4,5]:
    print "usage: %s lon lat height [file.bet]"
    exit(1)

def matmult(a,b):
    zip_b = zip(*b)
    return [[sum(a*b for a,b in zip(ra, cb)) for cb in zip_b] for ra in a]

values = list()

lons = float(sys.argv[1])
lats = float(sys.argv[2])
height = float(sys.argv[3])

r_earth = 6378137
m_lon = (2*pi/360) * r_earth * cos(lats*pi/180)
m_lat = (2*pi/360) * r_earth

if len(sys.argv) == 5:
    fd = open(sys.argv[4])
else:
    fd = sys.stdin

for line in fd:
    t,lon,lat,h,r,p,y = line.split()[:7]
    try:
        values.append((float(t),float(lon)/100000,float(lat)/100000,float(h),float(r)*pi/180,float(p)*pi/180,float(y)*pi/180))
    except:
        pass

fd.close()

import numpy as np

for t,lon,lat,h,r,p,y in values:
    # given in row-major order
    rx = lambda y: np.matrix([[      1,      0,      0],
                              [      0, cos(y),-sin(y)],
                              [      0, sin(y), cos(y)]])
    ry = lambda y: np.matrix([[ cos(y),      0, sin(y)],
                              [      0,      1,      0],
                              [-sin(y),      0, cos(y)]])
    rz = lambda y: np.matrix([[ cos(y),-sin(y),      0],
                              [ sin(y), cos(y),      0],
                              [     0,       0,      1]])
    # zxz, zyz, xyz, yxz
    r = ry(r)*rx(-p)*rz(-y)
    f = list(r.flat)
    inMatrix = f[0:3]+[m_lon*(lon-lons)]+f[3:6]+[m_lat*(lat-lats)]+f[6:9]+[h-height]+[0,0,0,1]
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
    #print "%f %f %f 0 %f %f %f 0 %f %f %f 0 %f %f %f 1 2"%(
    #        r[0][0], r[1][0], r[2][0],
    #        r[0][1], r[1][1], r[2][1],
    #        r[0][2], r[1][2], r[2][2],
    #        -x1,x2+20,x0)
