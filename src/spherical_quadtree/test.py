#!/usr/bin/env python3

from math import sin, cos, asin, acos, sqrt, pi, atan2

x, y, z = (1,0,0)

def angle_cosines(p, q):
    return acos(sin(p[0])*sin(q[0])+cos(p[0])*cos(q[0])*cos(q[1]-p[1]))

def angle_haversines(p, q):
    return 2*asin(sqrt(sin((q[0]-p[0])*0.5)**2 + cos(p[0])*cos(q[0])*sin(0.5*(q[1]-p[1]))**2))

def angle_cart(p, q):
    return acos(p[0]*q[0]+p[1]*q[1]+p[2]*q[2])

def cart2spher(v):
    x, y, z = v
    # theta: -0.5*pi ... 0.5*pi
    # phi: 0 .. 2*pi
    theta = acos(z)
    phi = atan2(y, x)
    if theta < -0.5*pi:
        raise Exception("theta is %f" % theta)
    if theta > 0.5*pi:
        raise Exception("theta is %f" % theta)
    if phi < 0:
        raise Exception("phi is %f" % phi)
    if phi > 2*pi:
        raise Exception("phi is %f" % phi)
    return theta, phi

def cart2geo(v):
    x, y, z = v
    # theta is latitude (polar angle)
    # we subtract from 0.5*pi to get the angle between the equator and the
    # vector instead of the angle between the vector and the azimuth
    theta = 0.5*pi - acos(z) # no need to divide by r because r == 1.0
    # phi is longitude (azimuth)
    phi = atan2(y,x)
    # make sure that the longitude is between -180 and 180 degrees
    phi -= pi
    if phi < -pi:
        phi += 2*pi
    # first theta, then phi as by ISO 80000-2:2009 and ISO 31-11 which define a
    # right-handed coordinate system (r,theta,phi)
    # latitude, longitude
    return theta,phi

def spher2cart(v):
    theta, phi = v
    x = sin(theta)*cos(phi)
    y = sin(theta)*sin(phi)
    z = cos(theta)
    return x, y, z

def geo2cart(v):
    theta, phi = v
    theta = 0.5*pi - theta
    phi += pi
    x = sin(theta)*cos(phi)
    y = sin(theta)*sin(phi)
    z = cos(theta)
    return x, y, z

def normalize(v):
    l = sqrt(v[0]*v[0]+v[1]*v[1]+v[2]*v[2])
    return v[0]/l, v[1]/l, v[2]/l

#p = cart2spher((x, y, z))
#q1 = p[0]-0.1, p[1]-0.1
#q2 = (pi-q1[0], pi+q1[1])
#
#print("p (cart):  %f, %f, %f" % (x, y, z))
#print("p (spher): %f, %f" % p)
#print("q1 (spher): %f, %f" % q1)
#print("q2 (spher): %f, %f" % q2)
#
#print("angle p/q1 law cosines:    %f" % angle_cosines(p, q1))
#print("angle p/q2 law cosines:    %f" % angle_cosines(p, q2))
#print("angle p/q1 law haversines: %f" % angle_haversines(p, q1))
#print("angle p/q2 law haversines: %f" % angle_haversines(p, q2))
#
#q1 = spher2cart(q1)
#q2 = spher2cart(q2)
#print("q1 (cart): %f, %f, %f" % q1)
#print("q2 (cart): %f, %f, %f" % q2)
#
#print("angle p/q1 (cartesian):    %f" % angle_cart((x, y, z), q1))
#print("angle p/q1 (cartesian):    %f" % angle_cart((x, y, z), q2))


p = (0.12, 0.8)
q = (0.3, 0.14)

def middle1(p, q):
    p = geo2cart(p)
    q = geo2cart(q)
    m = (q[0]+p[0])/2, (q[1]+p[1])/2, (q[2]+p[2])/2
    m = normalize(m)
    return cart2geo(m)

# convert geo to cartesian and average the two vectors
# 4*sin
# 4*cos
# 1*acos
# 1*sqrt
# 1*atan2
def middle2(p, q):
    a = sin(0.5*pi - p[0])
    b = sin(0.5*pi - q[0])
    x1 = a*cos(p[1] + pi)
    y1 = a*sin(p[1] + pi)
    z1 = cos(0.5*pi - p[0])
    x2 = b*cos(q[1] + pi)
    y2 = b*sin(q[1] + pi)
    z2 = cos(0.5*pi - q[0])
    x, y, z = (x1+x2)/2, (y1+y2)/2, (z1+z2)/2
    l = sqrt(x*x+y*y+z*z)
    x, y, z = x/l, y/l, z/l
    theta = 0.5*pi - acos(z)
    phi = atan2(y,x) - pi
    if phi < -pi:
        phi += 2*pi
    return theta,phi

# rotate one of the vectors by half the angle between them
# 1 x acos
# 1 x sqrt
# 2 x cos
# 1 x sin
def middle3(p, q):
    angle = acos(p[0]*q[0]+p[1]*q[1]+p[2]*q[2])/2
    u = p[1]*q[2] - p[2]*q[1]
    v = p[2]*q[0] - p[0]*q[2]
    w = p[0]*q[1] - p[1]*q[0]
    l = sqrt(u*u+v*v+w*w)
    u, v, w = u/l, v/l, w/l
    x, y, z = p
    f = (u*x+v*y+w*z)*(1-cos(angle))
    s = sin(angle)
    c = cos(angle)
    i = u*f+x*c+(-w*y+v*z)*s
    j = v*f+y*c+(w*x-u*z)*s
    k = w*f+z*c+(-v*x+u*y)*s
    return cart2geo((i,j,k))

def middle4(p, q):
    m = (p[0]+q[0])/2, (p[1]+q[1])/2, (p[2]+q[2])/2
    l = sqrt(m[0]*m[0] + m[1]*m[1] + m[2]*m[2])
    return cart2geo((m[0]/l, m[1]/l, m[2]/l))

#print(middle1(p, q))
#print(middle2(p, q))
#
#p = geo2cart(p)
#q = geo2cart(q)
#print(middle3(p, q))
#print(middle4(p, q))

mainvertices = []
for x in [-1,1]:
    for y in [-1,1]:
        for z in [-1,1]:
            # make sure that the vertices of the triangle are given in the same
            # order
            # i.e. make sure that the normal vector always points outward
            if (x > 0) ^ (y > 0) ^ (z > 0) == True:
                v1 = (x,0,0)
                v2 = (0,y,0)
                v3 = (0,0,z)
            else:
                v1 = (0,0,z)
                v2 = (0,y,0)
                v3 = (x,0,0)
            mainvertices.append((v1,v2,v3))

def tripleproduct(a, b, c):
    u = a[1]*b[2] - a[2]*b[1]
    v = a[2]*b[0] - a[0]*b[2]
    w = a[0]*b[1] - a[1]*b[0]
    return u*c[0]+v*c[1]+w*c[2]

for x,y,z in [(-0.5,0.5,0.5),(0.5,-0.5,0.5),(0.5,-0.5,-0.5)]:
    v1,v2,v3 = mainvertices[ int(x > 0) << 2 | int(y > 0) << 1 | int(z > 0) ]
    print(tripleproduct(v1,v2,(x,y,z)))
    print(tripleproduct(v2,v3,(x,y,z)))
    print(tripleproduct(v3,v1,(x,y,z)))
