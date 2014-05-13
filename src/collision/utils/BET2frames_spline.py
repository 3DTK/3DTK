#!/usr/bin/env python

import sys

if len(sys.argv) != 5:
    print "usage: %s lon lat height file.bet"
    exit(1)

lons = float(sys.argv[1])
lats = float(sys.argv[2])
height = float(sys.argv[3])

r_earth = 6378137
m_lon = (2*pi/360) * r_earth * cos(lats*pi/180)
m_lat = (2*pi/360) * r_earth

with open(sys.argv[4]) as f:
    for line in f:
        t,lon,lat,h,r,p,y = line.split()[:7]
        try:
            values.append((float(t),m_lon*(float(lon)/100000-lons),m_lat*(float(lat)/100000-lats),float(h)-height,float(r)*pi/180,float(p)*pi/180,float(y)*pi/180))
        except:
            pass


