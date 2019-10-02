#!/usr/bin/python3

import os
import sys
from math import acos, pi, sqrt, atan2, sin, cos, asin
import random
import time

sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..', 'lib'))
try:
	import py3dtk
except ImportError:
	print("Cannot find py3dtk module. Try recompiling 3dtk with WITH_PYTHON set to ON", file=sys.stderr)
	exit(1)

# return the length of a vector
def length(v):
    return sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2])

# normalize a vector
def norm(v):
    l = length(v)
    return v[0]/l, v[1]/l, v[2]/l

# calculate the cross product
def cross(a, b):
    u = a[1]*b[2] - a[2]*b[1]
    v = a[2]*b[0] - a[0]*b[2]
    w = a[0]*b[1] - a[1]*b[0]
    return u,v,w

# calculate the triple product between three 3D vectors
def tripleproduct(a, b, c):
    u,v,w = cross(a, b)
    return u*c[0]+v*c[1]+w*c[2]

# calculate the mid-point between two points on the surface of a unit sphere
def middle(a, b, vertices, middlemap):
    i = middlemap.get((a,b))
    if i is not None:
        return i
    p = vertices[a]
    q = vertices[b]
    m = (p[0]+q[0])/2, (p[1]+q[1])/2, (p[2]+q[2])/2
    vertices.append(norm(m))
    i = len(vertices) - 1
    middlemap[(a,b)] = i
    return i

# calculate the circumcircle of a triangle in 3D and return the unit vector of
# the center of the circumcircle as well as the angle theta of the sphere cap
# that has the circumcirle as its base
def circumcircle(v1,v2,v3):
    a = v1[0] - v3[0], v1[1] - v3[1], v1[2] - v3[2]
    b = v2[0] - v3[0], v2[1] - v3[1], v2[2] - v3[2]
    c = a[0] - b[0], a[1] - b[1], a[2] - b[2]
    la = length(a)
    la2 = la**2
    lb = length(b)
    lb2 = lb**2
    axb = cross(a,b)
    laxb = length(axb)
    laxb2 = 2*laxb**2
    r = (la*lb*length(c))/(2*laxb)
    # Now we have the radius of the circumcircle but in the end, instead of the
    # radius, we want half the angle under which the circumcircle is seen from
    # the center of the sphere.
    # In other words: instead of the radius of the base of the spherical cap,
    # we want the angle theta
    theta = asin(r)
    sa = lb2*a[0], lb2*a[1], lb2*a[2]
    sb = la2*b[0], la2*b[1], la2*b[2]
    di = sb[0]-sa[0], sb[1]-sa[1], sb[2]-sa[2]
    p = cross(di, axb)
    p = p[0]/laxb2 + v3[0], p[1]/laxb2 + v3[1], p[2]/laxb2 + v3[2]
    # the point p we are interested in is not the center of the base of the
    # circle cap but the point on the unit-sphere so that we can easily find
    # the angle between it and other vectors later
    p = norm(p)
    return p, theta

# geographic coordinates from a unit-vector
# note that these are not spherical coordinates, as those calculate the
# latitudal angle (or phi or polar angle) as the angle from the azimuth and not
# as the angle from the equator
# but to apply the laws of cosine or haversine we need the polar angle to be
# measured from the equator
def cart2geo(v):
    x, y, z = v
    # theta is latitude (polar angle)
    # we subtract from 0.5*pi to get the angle between the equator and the
    # vector instead of the angle between the vector and the azimuth
    theta = 0.5*pi - acos(z) # no need to divide by r because r == 1.0
    # phi is longitude (azimuth)
    phi = atan2(y,x)
    # make sure that the longitude is between -180 and 180 degrees
    # this normalization step is optional
    phi -= pi
    if phi < -pi:
        phi += 2*pi
    # first theta, then phi as by ISO 80000-2:2009 and ISO 31-11 which define a
    # right-handed coordinate system (r,theta,phi)
    # latitude, longitude
    return theta,phi

# inspiring work for spherical quad trees:
#
# "Navigating through Triangle Meshes Implemented as Linear Quadtrees" by Michael Lee and Hanan Samet
# "Traversing the Triangle Elements of an Icosahedral Spherical Representation in Constant-Time" by Michael Lee and Hanan Samet
# "Indexing the Sphere with the Hierarchical Triangular Mesh" by Alexander S. Szalay, Jim Gray, George Fekete, Peter Z. Kunszt, Peter Kukol, Ani Thakar
# "SEARCHABLE SKY COVERAGE OF ASTRONOMICAL OBSERVATIONS: FOOTPRINTS AND EXPOSURES" by Tamás Budavári, Alexander S. Szalay and György Fekete
# "A Hierarchical Spatial Data Structure for Global Geographic Information Systems" by Michael F. Goodchild and Yang Shiren
# "Rendering and managing spherical data with sphere quadtrees" by György Fekete
# "Comparing Geometrical Properties of Global Grids" by A. Jon Kimerling, Kevin Sahr, Denis White, and Lian Song
# "Geodesic Discrete Global Grid Systems" by Kevin Sahr, Denis White, and A. Jon Kimerling
#
class QuadNode:
    def __init__(self, v1, v2, v3, indices, pts, vertices, middlemap):
        self.pts = pts
        # nodes do not need access to the triangle vertices for searching
        # We only store them to dump the vertices in PLY format later
        self.v1 = v1
        self.v2 = v2
        self.v3 = v3
        w1 = vertices[v1]
        w2 = vertices[v2]
        w3 = vertices[v3]
        self.ccp, self.ccr = circumcircle(w1,w2,w3)
        if len(indices) <= 100:
            # make this a leaf node
            self.t1 = None
            self.t2 = None
            self.t3 = None
            self.t4 = None
            self.indices = indices
            return
        self.indices = None
        v4 = middle(v1,v2, vertices, middlemap)
        v5 = middle(v2,v3, vertices, middlemap)
        v6 = middle(v3,v1, vertices, middlemap)
        w4 = vertices[v4]
        w5 = vertices[v5]
        w6 = vertices[v6]
        indices1 = []
        indices2 = []
        indices3 = []
        indices4 = []
        for i in indices:
            p = self.pts[i]
            # this version is nearly functionally identical to the more
            # computationally expensive version below except that the rare
            # situation that a point does not fit into any of the four
            # triangles due to floating point inaccuracy cannot arise
            # anymore.
            if tripleproduct(w4,w6,p) >= 0:
                indices1.append(i)
            elif tripleproduct(w5,w4,p) >= 0:
                indices2.append(i)
            elif tripleproduct(w6,w5,p) >= 0:
                indices3.append(i)
            else:
                indices4.append(i)
            ## we must use >= or otherwise points exactly on the triangle
            ## borders will not be matched by any triangle.
            #if tripleproduct(w1,w4,p) >= 0 and tripleproduct(w4,w6,p) >= 0 and tripleproduct(w6,w1,p) >= 0:
            #    indices1.append(i)
            #elif tripleproduct(w2,w5,p) >= 0 and tripleproduct(w5,w4,p) >= 0 and tripleproduct(w4,w2,p) >= 0:
            #    indices2.append(i)
            #elif tripleproduct(w3,w6,p) >= 0 and tripleproduct(w6,w5,p) >= 0 and tripleproduct(w5,w3,p) >= 0:
            #    indices3.append(i)
            #elif tripleproduct(w4,w5,p) >= 0 and tripleproduct(w5,w6,p) >= 0 and tripleproduct(w6,w4,p) >= 0:
            #    indices4.append(i)
            #else:
            #    raise Exception("impossible for %f %f %f"%p)
        self.t1 = QuadNode(v1,v4,v6,indices1, pts, vertices, middlemap)
        self.t2 = QuadNode(v2,v5,v4,indices2, pts, vertices, middlemap)
        self.t3 = QuadNode(v3,v6,v5,indices3, pts, vertices, middlemap)
        self.t4 = QuadNode(v4,v5,v6,indices4, pts, vertices, middlemap)

    def __str__(self):
        if self.indices is not None:
            return "%d" % len(self.indices)
        else:
            return "(%s, %s, %s, %s)" % (str(self.t1),str(self.t2),str(self.t3),str(self.t4))


    # an overview of all possible configurations of a triangle and a search
    # radius is given on page 14 of:
    #
    # "Indexing the Sphere with the Hierarchical Triangular Mesh" by Alexander S. Szalay, Jim Gray, George Fekete, Peter Z. Kunszt, Peter Kukol, Ani Thakar
    def search(self, p, r):
        if self.indices is not None:
            # this is a leaf node, so we check all the points for whether they
            # fulfill the angular distance criterion r from the query point p
            res = []
            for i in self.indices:
                q = self.pts[i]
                dot = p[0]*q[0]+p[1]*q[1]+p[2]*q[2]
                # The dot product of the vector with itself might lead to a
                # number slightly greater than 1.0 due to floating point
                # inaccuracies.
                if dot >= 1.0:
                    res.append(i)
                    continue
                angle = acos(dot)
                if angle < r:
                    res.append(i)
            return res
        dot = p[0]*self.ccp[0] + p[1]*self.ccp[1] + p[2]*self.ccp[2]
        angle = acos(dot)
        if angle > r+self.ccr:
            # circumcircle of this triangle is completely disjunct from the
            # search radius
            return []
        if angle < r-self.ccr:
            # circumcircle of this triangle is completely inside the search
            # radius
            return self.getall()
        # circumcircle of the triangle intersects with the search radius, so
        # continue searching in all children
        res = []
        res.extend(self.t1.search(p,r))
        res.extend(self.t2.search(p,r))
        res.extend(self.t3.search(p,r))
        res.extend(self.t4.search(p,r))
        return res

    def getall(self):
        if self.indices is not None:
            return self.indices
        res = []
        res.extend(self.t1.getall())
        res.extend(self.t2.getall())
        res.extend(self.t3.getall())
        res.extend(self.t4.getall())
        return res

class QuadTree:
    def __init__(self, pts):
        # project all points onto a unit sphere
        self.pts = [ norm(v) for v in pts ]

        # create a list containing the eight sides of the octahedron
        #
        # we choose the octahedron because it is trivial to check whether a
        # point falls into one of the faces by aligning the octahendron with
        # the coordinate axis (it then boils down to a check of the signs) and
        # because the fewer faces, the fewer triangle checks have to be done to
        # figure out into which face a point falls
        self.vertices = [(-1,0,0),(1,0,0,),(0,-1,0),(0,1,0),(0,0,-1),(0,0,1)]
        mainvertices = []
        for x in [-1,1]:
            for y in [-1,1]:
                for z in [-1,1]:
                    v1 = 0 if x < 0 else 1
                    v2 = 2 if y < 0 else 3
                    v3 = 4 if z < 0 else 5
                    # make sure that the vertices of the triangle are given in the same
                    # order
                    # i.e. make sure that the normal vector always points outward
                    if (x > 0) ^ (y > 0) ^ (z > 0) == False:
                        v1, v3 = v3, v1
                    mainvertices.append((v1,v2,v3))

        buckets = [[],[],[],[],[],[],[],[]]
        print("sorting points into %d areas" % len(buckets), file=sys.stderr)
        before = time.time()
        for i,(x,y,z) in enumerate(self.pts):
            idx = int(x > 0) << 2 | int(y > 0) << 1 | int(z > 0)
            ## the following is just a check to make sure that our assignment
            ## algorithm works. It can be commented out after it has been verified
            ## that the algorithm works.
            #v1,v2,v3 = mainvertices[idx]
            #if tripleproduct(v1,v2,(x,y,z)) > 0 and tripleproduct(v2,v3,(x,y,z)) > 0 and tripleproduct(v3,v1,(x,y,z)) > 0:
            #    pass
            #else:
            #    raise Exception("duck")
            buckets[idx].append(i)
        after = time.time()
        print("took: %f" % (after-before), file=sys.stderr)
        middlemap = dict()
        print("constructing trees", file=sys.stderr)
        before = time.time()
        self.trees = [ QuadNode(v1,v2,v3,b,self.pts,self.vertices,middlemap) for b,(v1,v2,v3) in zip(buckets,mainvertices)]
        after = time.time()
        print("took: %f" % (after-before), file=sys.stderr)

    def search(self, p, radius):
        res = []
        for t in self.trees:
            res.extend(t.search(p,radius))
        return res

    def dump_ply(self):
        faces = list()
        for t in self.trees:
            def walk(t, l):
                if t.indices is not None or l == 0:
                    # if it is a child node, dump the vertex indices
                    faces.append((t.v1, t.v2, t.v3))
                    return
                walk(t.t1, l-1)
                walk(t.t2, l-1)
                walk(t.t3, l-1)
                walk(t.t4, l-1)
            walk(t, 100)
        print("ply")
        print("format ascii 1.0")
        print("element vertex", len(self.vertices))
        print("property float x")
        print("property float y")
        print("property float z")
        print("element face", len(faces))
        print("property list uchar int vertex_indices")
        print("end_header")
        for v in self.vertices:
            print("%f %f %f"%v)
        for f in faces:
            print("3 %d %d %d"%f)

def main():
    fmt = py3dtk.IOType.UOS
    start = end = 0
    scanserver = False
    print("reading in scans...", file=sys.stderr)
    #py3dtk.openDirectory(scanserver, '../wü_city/3d', fmt, start, end)
    py3dtk.openDirectory(scanserver, '/home/josch/work/publications/isprs2017/data/underwood/sim/3d', fmt, start, end)
    #py3dtk.openDirectory(scanserver, '/home/3dtk/slam6d/deleteme', fmt, start, end)
    if len(py3dtk.allScans) != 1:
        raise Exception("expected only one scan")
    # put all points at the surface of the unit sphere
    #xyz = [ (s,-u,o) for u,o,s in py3dtk.DataXYZ(py3dtk.allScans[0].get("xyz")) ]
    #xyz = [ norm(v) for v in xyz ]
    xyz = [ norm(v) for v in py3dtk.DataXYZ(py3dtk.allScans[0].get("xyz")) ]
    tp = [ cart2geo(v) for v in xyz ]
    random.seed(0)
    before = time.time()
    qtree = QuadTree(py3dtk.DataXYZ(py3dtk.allScans[0].get("xyz")))
    after = time.time()
    print("building py QuadTree took", after-before, file=sys.stderr)
    before = time.time()
    ctree = py3dtk.QuadTree(py3dtk.DataXYZ(py3dtk.allScans[0].get("xyz")))
    after = time.time()
    print("building C++ QuadTree took", after-before, file=sys.stderr)
    success = True
    for i in range(1000):
        p = random.choice(xyz)
        # find all points within 1 to 5 degree of that point
        radius = random.randint(1,5) * (2*pi)/360
        res = []
        print("finding neighbors of (%f, %f, %f) with r = %f" % (p[0],p[1],p[2],radius), file=sys.stderr)
        before = time.time()
        for q in xyz:
            dot = p[0]*q[0]+p[1]*q[1]+p[2]*q[2]
            # the dot product might be greater than 1.0 due to floating point inaccuracies
            if dot >= 1.0:
                res.append(q)
                continue
            angle = acos(dot)
            if angle < radius:
                #print("%f %f" %cart2geo(q))
                res.append(q)
        after = time.time()
        t_bf = after-before
        # now do the same test but with geographic coordinates
        # these checks are really slow, so we don't time them
        pg = cart2geo(p)
        res2 = []
        for q in tp:
            # law of cosines
            dot = sin(pg[0])*sin(q[0])+cos(pg[0])*cos(q[0])*cos(q[1]-pg[1])
            # numerical problems might make the result of the above greater
            # than 1.0 if pg is equal to q
            if dot >= 1.0:
                res2.append(q)
                continue
            angle = acos(dot)
            if angle < radius:
                #print("%f %f" %(q[0], q[1]))
                res2.append(q)
        res3 = []
        for q in tp:
            # laws of haversines
            angle = 2*asin(sqrt(sin((q[0]-pg[0])*0.5)**2 + cos(pg[0])*cos(q[0])*sin(0.5*(q[1]-pg[1]))**2))
            if angle < radius:
                #print("%f %f" %(q[0], q[1]))
                res3.append(q)
        res4 = []
        # finally, use the quad-tree to do the searches
        before = time.time()
        res4 = [xyz[i] for i in qtree.search(p, radius)]
        after = time.time()
        t_qtree = after-before
        # and the ctree
        before = time.time()
        res5 = [xyz[i] for i in ctree.search(p, radius)]
        after = time.time()
        t_ctree = after-before
        # to be able to compare the sets, we must convert the lists in
        # cartesian coordinates to geo coordinates. Converting the geo lists to
        # cartesian would involve some conversion errors
        res = [ cart2geo(v) for v in res ]
        res4 = [ cart2geo(v) for v in res4 ]
        res5 = [ cart2geo(v) for v in res5 ]
        if len(res) != len(res5):
            print("lengths differ. %d vs. %d" % (len(res), len(res5)), file=sys.stderr)
            print("points in one but not the other: %s" % str(set(res) ^ set(res5)), file=sys.stderr)
            print("# ", end="")
            success = False
        if len(res) != len(res4):
            print("lengths differ. %d vs. %d" % (len(res), len(res4)), file=sys.stderr)
            print("points in one but not the other: %s" % str(set(res) ^ set(res4)), file=sys.stderr)
            print("# ", end="")
            success = False
        if len(res) != len(res2):
            print("lengths differ. %d vs. %d" % (len(res), len(res2)), file=sys.stderr)
            print("points in one but not the other: %s" % str(set(res) ^ set(res2)), file=sys.stderr)
            print("# ", end="")
            success = False
        if len(res) != len(res3):
            print("lengths differ. %d vs. %d" % (len(res), len(res3)), file=sys.stderr)
            print("points in one but not the other: %s" % str(set(res) ^ set(res3)), file=sys.stderr)
            print("# ", end="")
            success = False
        if sorted(res) != sorted(res5):
            print("content differs", file=sys.stderr)
            print("points in one but not the other: %s" % str(set(res) ^ set(res5)), file=sys.stderr)
            print("# ", end="")
            success = False
        if sorted(res) != sorted(res4):
            print("content differs", file=sys.stderr)
            print("points in one but not the other: %s" % str(set(res) ^ set(res4)), file=sys.stderr)
            print("# ", end="")
            success = False
        if sorted(res) != sorted(res2):
            print("content differs", file=sys.stderr)
            print("points in one but not the other: %s" % str(set(res) ^ set(res2)), file=sys.stderr)
            print("# ", end="")
            success = False
        if sorted(res) != sorted(res3):
            print("content differs", file=sys.stderr)
            print("points in one but not the other: %s" % str(set(res) ^ set(res3)), file=sys.stderr)
            print("# ", end="")
            success = False
        print("%d %f %f %f" % (i, t_bf, t_qtree, t_ctree))
    if success == False:
        exit(1)
    #with open("scan008.3d", "w") as f:
    #    for x,y,z in res4:
    #        f.write("%f %f %f\n" % (x*100,y*100,z*100))


if __name__ == '__main__':
    main()

# 2.753392311621602 -0.36393990636763207
# 0.388200341968191  2.7776527472221613
