#!/usr/bin/python3

import py3dtk
from math import sqrt

query = (10,0,0)
dist2 = 4
def dist(p):
    a = query[0] - p[0]
    b = query[1] - p[1]
    c = query[2] - p[2]
    return sqrt(a*a+b*b+c*c)

py3dtk.openDirectory(False, "dat", py3dtk.IOType.UOS, 0, -1)
points=list()
for s in py3dtk.allScans:
    xyz = py3dtk.DataXYZ(s.getByString("xyz"))
    points.extend(xyz)

kdtree = py3dtk.KDtreeIndexed(points)
res = kdtree.fixedRangeSearch(query, dist2)

res = sorted([points[i] for i in res], key=dist)
print(["%.03f"%dist(p) for p in res])
