#!/usr/bin/python3

from collections import defaultdict
import sys
import math
import argparse
from multiprocessing import Pool
import os
from functools import cmp_to_key

sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..', 'lib'))
try:
    import py3dtk
except ImportError:
    print("Cannot find py3dtk module. Try recompiling 3dtk with WITH_PYTHON set to ON", file=sys.stderr)
    exit(1)

sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..', 'src', 'spherical_quadtree'))
import spherical_quadtree as sq

def voxel_of_point(point, voxel_size):
    return int(point[0]//voxel_size), int(point[1]//voxel_size), int(point[2]//voxel_size)

# walk voxels as described in
#   Fast Voxel Traversal Algorithm for Ray Tracing
#   by John Amanatides, Andrew Woo
#   Eurographics ’87
#   http://www.cs.yorku.ca/~amana/research/grid.pdf
def walk_voxels(start, end, voxel_size, voxel_occupied_by_slice, current_slice, max_search_distance, diff):
    #print("from: %f %f %f" % start)
    #print("to: %f %f %f" % end)
    if max_search_distance == 0:
        return set()
    direction = (end[0] - start[0], end[1] - start[1], end[2] - start[2])
    dist = math.sqrt(direction[0]*direction[0]+direction[1]*direction[1]+direction[2]*direction[2])
    # compute the t value representing the desired search radius
    if max_search_distance is None:
        tMax = 1.0
    else:
        tMax = max_search_distance/dist
        if tMax > 1.0:
            tMax = 1.0
    startX, startY, startZ = X, Y, Z = voxel_of_point(start, voxel_size)
    endX, endY, endZ = voxel_of_point(end, voxel_size)
    #print("start: %d %d %d" % (X, Y, Z))
    #print("end: %d %d %d" % (endX, endY, endZ))
    #print("direction: %f %f %f" % direction)
    # tMax*: value t at which the segment crosses the first voxel boundary in the given direction
    # stepX: in which direction to increase the voxel count (1 or -1)
    # tDelta*: value t needed to span the voxel size in the given direction
    if direction[0] == 0:
        tDeltaX = None
        stepX = None
        tMaxX = math.inf
    else:
        stepX = 1 if direction[0] > 0 else -1
        tDeltaX = stepX*voxel_size/direction[0]
        tMaxX = tDeltaX * (1.0 - (stepX*start[0]/voxel_size) % 1)
    if direction[1] == 0:
        tDeltaY = None
        stepY = None
        tMaxY = math.inf
    else:
        stepY = 1 if direction[1] > 0 else -1
        tDeltaY = stepY*voxel_size/direction[1]
        tMaxY = tDeltaY * (1.0 - (stepY*start[1]/voxel_size) % 1)
    if direction[2] == 0:
        tDeltaZ = None
        stepZ = None
        tMaxZ = math.inf
    else:
        stepZ = 1 if direction[2] > 0 else -1
        tDeltaZ = stepZ*voxel_size/direction[2]
        tMaxZ = tDeltaZ * (1.0 - (stepZ*start[2]/voxel_size) % 1)
    # if we step into negative direction and the starting point happened to fall
    # exactly onto the voxel boundary, then we need to reset the respective
    # tMax value as we'd otherwise skip the first voxel in that direction
    if stepX == -1 and tMaxX == tDeltaX:
        tMaxX = 0.0
    if stepY == -1 and tMaxY == tDeltaY:
        tMaxY = 0.0
    if stepZ == -1 and tMaxZ == tDeltaZ:
        tMaxZ = 0.0
    #print("steps: %d %d %d" % (stepX, stepY, stepZ))
    #print("deltas: %f %f %f" % (tDeltaX, tDeltaY, tDeltaZ))
    #print("max: %f %f %f" % (tMaxX, tMaxY, tMaxZ))
    # in contrast to the original algorithm by John Amanatides and Andrew 
    # Woo we increment a counter and multiply the step size instead of
    # adding up the steps. Doing the latter might introduce errors because
    # due to floating point precision errors, 0.1+0.1+0.1 is unequal 3*0.1.
    multX = multY = multZ = 0
    tMaxXStart = tMaxX
    tMaxYStart = tMaxY
    tMaxZStart = tMaxZ
    empty_voxels = set()
    #i = 0
    # iterate until either:
    #  - the final voxel is reached
    #  - tMax is reached by all tMax-coordinates
    #  - the current voxel contains points of (or around) the current scan
    while X != endX or Y != endY or Z != endZ:
        # We need an epsilon to support cases in which we end up
        # searching up to the target voxel (with tMax = 1.0) and the
        # target coordinate being exactly on the voxel boundary.
        #epsilon = sys.float_info.epsilon
        epsilon = 1e-13
        if tMaxX > 1.0+epsilon and tMaxY > 1.0+epsilon and tMaxZ > 1.0+epsilon:
            print(tMaxX, tMaxY, tMaxZ, start, end)
            raise Exception("this should never happen")
        if tMaxX > tMax-epsilon and tMaxY > tMax-epsilon and tMaxZ > tMax-epsilon:
            break
        #print("i: %d,\t%d %d %d != %d %d %d, %f %f %f" % (i, X, Y, Z, endX, endY, endZ, tMaxX, tMaxY, tMaxZ))
        #i += 1
        if tMaxX < tMaxY:
            if tMaxX < tMaxZ:
                multX += 1
                X = startX + multX*stepX
                tMaxX = tMaxXStart + multX*tDeltaX
            else:
                multZ += 1
                Z = startZ + multZ*stepZ
                tMaxZ = tMaxZStart + multZ*tDeltaZ
        else:
            if tMaxY < tMaxZ:
                multY += 1
                Y = startY + multY*stepY
                tMaxY = tMaxYStart + multY*tDeltaY
            else:
                multZ += 1
                Z = startZ + multZ*stepZ
                tMaxZ = tMaxZStart + multZ*tDeltaZ
        #print("%f %f %f" % (tMaxX, tMaxY, tMaxZ))
        #print("visiting voxel: %d %d %d"%(X,Y,Z))
        # if the voxel has no point in it at all, continue searching
        if (X, Y, Z) not in voxel_occupied_by_slice:
            # we don't need to add this voxel to the empty voxel list because it was
            # empty to begin with
            continue
        # The following implements a sliding window within which voxels
        # that also contain points with a similar index as the current
        # slice are not marked as free. Instead, the search aborts
        # early.
        # If no points around the current slice are found in the voxel,
        # then the points in it were seen from a very different scanner
        # position and thus, these points are actually not there and
        # the voxel must be marked as free.

        # if the voxel has a point in it, only abort if the slice number
        # is close to the current slice (use difference of 10 slices)
        #diff = 285 # full circle: 570
        #diff = 140
        #diff = 0
        if diff == 0:
            if current_slice not in voxel_occupied_by_slice[(X, Y, Z)]:
                #print("adding to empty voxels: %d %d %d"%(X,Y,Z))
                empty_voxels.add((X, Y, Z))
                continue
            break
        else:
            for slice_num in voxel_occupied_by_slice[(X, Y, Z)]:
                if slice_num >= current_slice - diff and slice_num <= current_slice + diff:
                    break
            else:
                # the loop went through without aborting, so the current voxel only
                # contains points from far away slices, so we continue searching
                empty_voxels.add((X, Y, Z))
                continue
            # the loop aborted early, so the current voxel does contain points around
            # the current scan slice and we abort
            break
    return empty_voxels
            

#def transform3(matrix, point):
#    x_neu = point[0] * matrix[0] + point[1] * matrix[4] + point[2] * matrix[8]
#    y_neu = point[0] * matrix[1] + point[1] * matrix[5] + point[2] * matrix[9]
#    z_neu = point[0] * matrix[2] + point[1] * matrix[6] + point[2] * matrix[10]
#    x_neu += matrix[12]
#    y_neu += matrix[13]
#    z_neu += matrix[14]
#    return (x_neu, y_neu, z_neu)

def formatname_to_io_type(string):
    if string.lower() == "uos":
        return py3dtk.IOType.UOS
    if string.lower() == "uosr":
        return py3dtk.IOType.UOSR
    if string.lower() == "xyz":
        return py3dtk.IOType.XYZ
    if string.lower() == "xyzr":
        return py3dtk.IOType.XYZR
    if string.lower() == "riegl_txt":
        return py3dtk.IOType.RIEGL_TXT
    if string.lower() == "rxp":
        return py3dtk.IOType.RXP

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("-s", "--start", type=int)
    parser.add_argument("-e", "--end", type=int)
    parser.add_argument("-f", "--format", type=formatname_to_io_type)
    parser.add_argument("--fuzz", type=float, default=0, help="How fuzzy the data is. I.e. how far points on a perfect plane are allowed to lie away from it in the scan.")
    parser.add_argument("--voxel-size", type=float, default=10)
    parser.add_argument("--diff", type=int, default=0, help="Number of scans before and after the current scan that are grouped together.")
    parser.add_argument("--no-subvoxel-accuracy", action='store_true', help="Do not calculate with subvoxel accuracy")
    parser.add_argument("--no-compute-normals", action='store_true', help="Do not compute surface offsets by their normal vectors")
    parser.add_argument("directory")
    args = parser.parse_args()

    voxel_size = args.voxel_size

    points_by_slice = list()
    trajectory = list()

    len_trajectory = args.end - args.start + 1
    print("directory: %s" % args.directory, file=sys.stderr)
    print("start: %d" % args.start, file=sys.stderr)
    print("end: %d" % args.end, file=sys.stderr)
    print("length: %d" % len_trajectory, file=sys.stderr)
    print("voxel size: %f" % voxel_size, file=sys.stderr)
    print("reading data...", file=sys.stderr)

    voxel_diagonal = math.sqrt(3*voxel_size*voxel_size)

    scanserver = False
    py3dtk.openDirectory(scanserver, args.directory, args.format, args.start, args.end)
    for i,s in enumerate(py3dtk.allScans, start=args.start):
        print("%f" % ((((i-args.start)+1)*100)/len_trajectory), end="\r", file=sys.stderr)
        # ignore points that are closer than a voxel diagonal
        s.setRangeFilter(-1, voxel_diagonal)
        xyz_orig = list(py3dtk.DataXYZ(s.get("xyz")))
        # transform all points into the global coordinate system
        s.transformAll(s.get_transMatOrg())
        # add the current position to the trajectory
        trajectory.append(s.get_rPos())
        # add all the points into their respective slices
        xyz = list(py3dtk.DataXYZ(s.get("xyz")))
        refl = py3dtk.DataReflectance(s.get("reflectance"))
        if len(xyz) != len(refl) or len(xyz_orig) != len(refl):
            print("unequal lengths %d %d in %d" % (len(xyz), len(refl), i))
            exit(1)
        points = list(zip(xyz,xyz_orig,refl))
        if not args.no_compute_normals:
            # sort points by their distance from the scanner for normal
            # computation later
            # Precompute the distances so that they are not computed multiple
            # times while sorting
            distances = { p:sq.length(p) for p in xyz_orig }
            points = sorted(points, key=cmp_to_key(lambda a,b: distances[a[1]] - distances[b[1]]))
        points_by_slice.append(points)
        print("number of points in scan %d: %d" % (i, len(points)), file=sys.stderr)

    print("calculate voxel occupation", file=sys.stderr)

    voxel_occupied_by_slice = defaultdict(set)

    for i, points in enumerate(points_by_slice):
        print("%f" % (((i+1)*100)/len_trajectory), end="\r", file=sys.stderr)
        for (x,y,z),_,_ in points:
            voxel_occupied_by_slice[voxel_of_point((x,y,z), voxel_size)].add(i)
    print("", file=sys.stderr)

    if len(voxel_occupied_by_slice) == 0:
        print("no voxel occupied", file=sys.stderr)
        exit(1)

    print("occupied voxels: %d" % len(voxel_occupied_by_slice), file=sys.stderr)

    print("find free voxels", file=sys.stderr)

    free_voxels = set()

    maxranges = []
    for i, pos in enumerate(trajectory):
        maxranges.append([None]*len(points_by_slice[i]))
    if not args.no_compute_normals:
        for i, pos in enumerate(trajectory):
            # build a quad tree
            print("building spherical quad tree", file=sys.stderr)
            qtree = sq.QuadTree([p for _,p,_ in points_by_slice[i]])
            print("building k-d tree", file=sys.stderr)
            kdtree = py3dtk.KDtree([p for _,p,_ in points_by_slice[i]])
            print("calculating ranges")
            # for each point in this scan (starting from the point closest to
            # the scanner) use its normal to calculate until when the line of
            # sight up to the point should be searched and apply the same limit
            # to all the points in its "shadow"
            for j,(_,p,_) in enumerate(points_by_slice[i]):
                # point was already assigned a maximum range
                if maxranges[i][j] is not None:
                    continue
                # distance of the point from the scanner
                k_nearest = kdtree.kNearestNeighbors(p, 20)
                normal, _ = py3dtk.calculateNormal(k_nearest)
                p_norm = sq.norm(p)
                # make sure that the normal vector points *toward* the scanner
                angle_cos = normal[0]*p_norm[0]+normal[1]*p_norm[1]+normal[2]*p_norm[2]
                # we don't need to calculate the acos to get the real angle
                # between the point vector and the normal because values less
                # than zero map to angles of more than 90 degrees
                if angle_cos >= 0:
                    # make sure that the normal turns toward the scanner
                    normal = (-1*normal[0], -1*normal[1], -1*normal[2])
                # we only want to traverse the lines of sight until they hit
                # the plane that lies a voxel diagonal above the current point
                # in normal direction
                # the base of that plane:
                p_base = (p[0]+normal[0]*(voxel_diagonal+args.fuzz),
                        p[1]+normal[1]*(voxel_diagonal+args.fuzz),
                        p[2]+normal[2]*(voxel_diagonal+args.fuzz))
                # the dividend should stay the same as it's only dependent on
                # the base of the plane
                dividend = p_base[0]*normal[0]+p_base[1]*normal[1]+p_base[2]*normal[2]
                # calculate the divisor for the current point
                divisor = p_norm[0]*normal[0]+p_norm[1]*normal[1]+p_norm[2]*normal[2]
                if dividend/divisor > sq.length(p):
                    raise Exception("p got lengthened: ", dividend/divisor)
                maxranges[i][j] = dividend/divisor
                # the scanner itself is situated close to the plane that p
                # is part of. Thus, shoot no ray to this point at all.
                if maxranges[i][j] < 0:
                    maxranges[i][j] = 0
                # points must not be too close or otherwise they will shadow
                # *all* the points
                if sq.length(p) < voxel_diagonal:
                    raise Exception("point too close to scanner")
                # now find all the points in the shadow of this one
                # the size of the shadow is determined by the angle under
                # which the voxel diagonal is seen at that distance
                angle = math.asin(voxel_diagonal/sq.length(p))
                for k in qtree.search(p_norm, angle):
                    p_k = points_by_slice[i][k][1]
                    p_k_norm = sq.norm(p_k)
                    divisor = p_k_norm[0]*normal[0]+p_k_norm[1]*normal[1]+p_k_norm[2]*normal[2]
                    d = dividend/divisor
                    # even though p_k is further away from the scanner than p
                    # and inside the shadow of p, it is still on top of or
                    # in front of (from the point of view of the scanner) of
                    # the plane that p is part of. Thus, we process this point
                    # later
                    if d > sq.length(p_k):
                        continue
                    # the scanner itself is situated close to the plane that p
                    # is part of. Thus, shoot no ray to this point at all.
                    if d < 0:
                        d = 0
                    # point is already covered by a closer one
                    if maxranges[i][k] is not None and maxranges[i][k] < d:
                        continue
                    maxranges[i][k] = d
            #with open("scan%03d.pose" % (i+2), "w") as f:
            #    print("%f %f %f"%pos, file=f)
            #    print("0 0 0", file=f)
            #with open("scan%03d.3d" % (i+2), "w") as f:
            #    for j,(_,p,_) in enumerate(points_by_slice[i]):
            #        maxrange = maxranges[i][j]
            #        d = tuple(p)
            #        r = sq.length(d)
            #        factor = maxrange/r
            #        if factor > 1:
            #            raise Exception("duck 2: %f" % factor)
            #        if factor < 0:
            #            raise Exception("duck 3: ", factor, maxrange, r)
            #        d = d[0]*factor, d[1]*factor, d[2]*factor
            #        print("%f %f %f 0" % d, file=f)

    print("walk voxels")
    for i, pos in enumerate(trajectory):
        #print("%f" % (((i+1)*100)/len_trajectory), end="\r", file=sys.stderr)
        for j,(p,_,_) in enumerate(points_by_slice[i]):
            #print("%f (%d)" % (((i+1)*100)/len_trajectory, j), end="\r", file=sys.stderr)
            free = walk_voxels(pos, p, voxel_size, voxel_occupied_by_slice, i, maxranges[i][j], args.diff)
            free_voxels |= free
        
    print("", file=sys.stderr)

    print("number of freed voxels: %d (%f %%)" % (len(free_voxels), 100*len(free_voxels)/len(voxel_occupied_by_slice)), file=sys.stderr)

    half_voxels = defaultdict(set)
    if not args.no_subvoxel_accuracy:
        print("calculate half-free voxels")
        for voxel in free_voxels:
            freed_slices = voxel_occupied_by_slice[voxel]
            neighbor_voxels = set([
                    (voxel[0]-1, voxel[1]-1, voxel[2]-1),
                    (voxel[0]-1, voxel[1]-1, voxel[2]+0),
                    (voxel[0]-1, voxel[1]-1, voxel[2]+1),
                    (voxel[0]-1, voxel[1]+0, voxel[2]-1),
                    (voxel[0]-1, voxel[1]+0, voxel[2]+0),
                    (voxel[0]-1, voxel[1]+0, voxel[2]+1),
                    (voxel[0]-1, voxel[1]+1, voxel[2]-1),
                    (voxel[0]-1, voxel[1]+1, voxel[2]+0),
                    (voxel[0]-1, voxel[1]+1, voxel[2]+1),
                    (voxel[0]+0, voxel[1]-1, voxel[2]-1),
                    (voxel[0]+0, voxel[1]-1, voxel[2]+0),
                    (voxel[0]+0, voxel[1]-1, voxel[2]+1),
                    (voxel[0]+0, voxel[1]+0, voxel[2]-1),
                    #(voxel[0]+0, voxel[1]+0, voxel[2]+0), this is the current voxel
                    (voxel[0]+0, voxel[1]+0, voxel[2]+1),
                    (voxel[0]+0, voxel[1]+1, voxel[2]-1),
                    (voxel[0]+0, voxel[1]+1, voxel[2]+0),
                    (voxel[0]+0, voxel[1]+1, voxel[2]+1),
                    (voxel[0]+1, voxel[1]-1, voxel[2]-1),
                    (voxel[0]+1, voxel[1]-1, voxel[2]+0),
                    (voxel[0]+1, voxel[1]-1, voxel[2]+1),
                    (voxel[0]+1, voxel[1]+0, voxel[2]-1),
                    (voxel[0]+1, voxel[1]+0, voxel[2]+0),
                    (voxel[0]+1, voxel[1]+0, voxel[2]+1),
                    (voxel[0]+1, voxel[1]+1, voxel[2]-1),
                    (voxel[0]+1, voxel[1]+1, voxel[2]+0),
                    (voxel[0]+1, voxel[1]+1, voxel[2]+1),
                    ])
            # remove voxels that were already marked as free from the neighbor
            neighbor_voxels -= free_voxels
            for neighbor in neighbor_voxels:
                # check if there is anything in the current neighbor
                if neighbor not in voxel_occupied_by_slice:
                    continue
                half_voxels[neighbor] |= freed_slices
        # make sure that no voxels are completely cleared
        half_voxels = {v:i for v,i in half_voxels.items() if i != voxel_occupied_by_slice[v]}

    print("write result", file=sys.stderr)
    with open("scan000.3d", "w") as f1, open("scan001.3d", "w") as f2:
        for i, points in enumerate(points_by_slice):
            print("%f" % (((i+1)*100)/len_trajectory), end="\r", file=sys.stderr)
            for (x,y,z),_,r in points:
                voxel = voxel_of_point((x,y,z), voxel_size)
                if voxel in free_voxels or i in half_voxels.get(voxel, set()):
                    # this point is in a voxel marked as free or is a point
                    # from a slice index that was freed in an adjacent voxel
                    f2.write("%s %s %s %s\n" % (x.hex(),y.hex(),z.hex(),r.hex()))
                else:
                    f1.write("%s %s %s %s\n" % (x.hex(),y.hex(),z.hex(),r.hex()))
    for pose in ["scan000.pose", "scan001.pose"]:
        with open(pose, "w") as f:
            f.write("0 0 0\n0 0 0\n");
    for frames in ["scan000.frames", "scan001.frames"]:
        with open(frames, "w") as f:
            f.write("1 0 0 0 0 1 0 0 0 0 1 0 0 0 0 1 2\n");
    print("", file=sys.stderr)
    print("done", file=sys.stderr)

if __name__ == "__main__":
    main()
