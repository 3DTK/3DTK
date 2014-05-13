#!/usr/bin/env python
# las2txt --parse xyzi --delimiter " " ../Tunnel_3Dthinn/scan001.las --stdout | ./lastxt2xyzr.py

import sys
from math import cos,pi

r_earth = 6378137

data = []

for line in sys.stdin:
    lon,lat,h,i = line.split()
    data.append((float(lon)/100000,float(lat)/100000,float(h),int(i)))

# find the average longitude
lons = [l[0] for l in data]
lons = sum(lons)/len(data)

# find the average latitude
lats = [l[1] for l in data]
lats = sum(lats)/len(data)

height = [l[2] for l in data]
height = sum(height)/len(data)

# meter per longitude depends on the latitude
m_lon = (2*pi/360) * r_earth * cos(lats*pi/180)
# meter per latitude is constant
m_lat = (2*pi/360) * r_earth

# now convert latitude and longitude to meters
# shift lat and lon so that they center around the avarage
for lon,lat,h,i in data:
    print m_lon*(lon-lons), m_lat*(lat-lats), h-height, i

sys.stderr.write("lon/lat/height: %f %f %f\n"%(lons,lats,height))
