#!/usr/bin/env python

from __future__ import print_function
import sys

data = list()

for line in sys.stdin:
    if line.startswith('#'):
        continue
    x,y,z,r = line.split()
    data.append((float(x),float(y),float(z),float(r)))

#com_x = sum([d[0] for d in data])/len(data)
#com_y = sum([d[1] for d in data])/len(data)
com_z = sum([d[2] for d in data])/len(data)
#print("%f %f %f"%(com_x, com_y, com_z), file=sys.stderr)
print("%f"%com_z, file=sys.stderr)

for x,y,z,r in data:
    print("%f %f %f %f"%(x,y,z-com_z,r))
