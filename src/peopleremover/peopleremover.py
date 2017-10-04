#!/usr/bin/python3

from collections import defaultdict
import sys
import math
import argparse
from multiprocessing import Pool
import os
from functools import cmp_to_key
import random

sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..', 'lib'))
try:
    import py3dtk
    import py3dtk.utils
except ImportError:
    print("Cannot find py3dtk module. Try recompiling 3dtk with WITH_PYTHON set to ON", file=sys.stderr)
    exit(1)

sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..', 'src', 'spherical_quadtree'))
import spherical_quadtree as sq

def voxel_of_point(point, voxel_size):
    return int(point[0]//voxel_size), int(point[1]//voxel_size), int(point[2]//voxel_size)

def visitor(voxel, *args, **kwargs):
    empty_voxels, voxel_occupied_by_slice, current_slice, diff = args
    # nothing to do here because the voxel is empty to begin with
    if voxel not in voxel_occupied_by_slice:
        return True

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

    if diff == 0:
        if current_slice not in voxel_occupied_by_slice[voxel]:
            empty_voxels.add(voxel)
            return True
        return False
    else:
        for slice_num in voxel_occupied_by_slice[voxel]:
            if slice_num >= current_slice - diff and slice_num <= current_slice + diff:
                break
        else:
            # the loop went through without aborting, so the current voxel only
            # contains points from far away slices, so we continue searching
            empty_voxels.add(voxel)
            return True
        # the loop aborted early, so the current voxel does contain points around
        # the current scan slice and we abort
        return False

# walk voxels as described in
#   Fast Voxel Traversal Algorithm for Ray Tracing
#   by John Amanatides, Andrew Woo
#   Eurographics ’87
#   http://www.cs.yorku.ca/~amana/research/grid.pdf
def walk_voxels(start, end, voxel_size, visitor, *args, **kwargs):
    direction = (end[0] - start[0], end[1] - start[1], end[2] - start[2])
    startX, startY, startZ = X, Y, Z = voxel_of_point(start, voxel_size)
    endX, endY, endZ = voxel_of_point(end, voxel_size)
    visitor((startX, startY, startZ), *args, **kwargs)
    if direction == (0, 0, 0):
        return
    if (startX, startY, startZ) == (endX, endY, endZ):
        return
    if direction[0] == 0:
        tDeltaX = None
        stepX = None
        tMaxX = math.inf
        maxMultX = math.inf
    else:
        stepX = 1 if direction[0] > 0 else -1
        tDeltaX = stepX*voxel_size/direction[0]
        tMaxX = tDeltaX * (1 - (stepX*start[0]/voxel_size) % 1)
        maxMultX = (endX - startX)*stepX
        if stepX == -1 and tMaxX == tDeltaX and startX != endX:
            X -= 1
            startX -= 1
            maxMultX -= 1
    if direction[1] == 0:
        tDeltaY = None
        stepY = None
        tMaxY = math.inf
        maxMultY = math.inf
    else:
        stepY = 1 if direction[1] > 0 else -1
        tDeltaY = stepY*voxel_size/direction[1]
        tMaxY = tDeltaY * (1 - (stepY*start[1]/voxel_size) % 1)
        maxMultY = (endY - startY)*stepY
        if stepY == -1 and tMaxY == tDeltaY and startY != endY:
            Y -= 1
            startY -= 1
            maxMultY -= 1
    if direction[2] == 0:
        tDeltaZ = None
        stepZ = None
        tMaxZ = math.inf
        maxMultZ = math.inf
    else:
        stepZ = 1 if direction[2] > 0 else -1
        tDeltaZ = stepZ*voxel_size/direction[2]
        tMaxZ = tDeltaZ * (1 - (stepZ*start[2]/voxel_size) % 1)
        maxMultZ = (endZ - startZ)*stepZ
        if stepZ == -1 and tMaxZ == tDeltaZ and startZ != endZ:
            Z -= 1
            startZ -= 1
            maxMultZ -= 1
    visitor((X, Y, Z), *args, **kwargs)
    if (X, Y, Z) == (endX, endY, endZ):
        return
    tMaxXStart = tMaxX
    tMaxYStart = tMaxY
    tMaxZStart = tMaxZ
    multX = multY = multZ = 0
    while True:
        minVal = min(tMaxX, tMaxY, tMaxZ)
        steppedX = steppedY = steppedZ = False
        if minVal == tMaxX:
            multX += 1
            X = startX + multX*stepX
            tMaxX = tMaxXStart + multX*tDeltaX
            steppedX = True
        if minVal == tMaxY:
            multY += 1
            Y = startY + multY*stepY
            tMaxY = tMaxYStart + multY*tDeltaY
            steppedY = True
        if minVal == tMaxZ:
            multZ += 1
            Z = startZ + multZ*stepZ
            tMaxZ = tMaxZStart + multZ*tDeltaZ
            steppedZ = True
        # If we end up stepping in more than one direction at the same time,
        # then we must also assess if we have to add the voxel that we just
        # "graced" in the process.
        # An additional voxel must only be added in six of the eight different
        # directions that we can step into. If we step in all positive or
        # in all negative directions, then no additional voxel is added.
        if ((steppedX and steppedY) or (steppedY and steppedZ) or (steppedX and steppedZ)) \
        and (stepX == 1 or stepY == 1 or stepZ == 1) and (stepX == -1 or stepY == -1 or stepZ == -1):
            addX, addY, addZ = X, Y, Z
            # a voxel was only possibly missed if we stepped into a
            # negative direction and if that step was actually carried
            # out in this iteration
            if steppedX:
                if stepX < 0:
                    if multX > maxMultX + 1:
                        break
                    addX += 1
                elif multX > maxMultX:
                    break
            if steppedY:
                if stepY < 0:
                    if multY > maxMultY + 1:
                        break
                    addY += 1
                elif multY > maxMultY:
                    break
            if steppedZ:
                if stepZ < 0:
                    if multZ > maxMultZ + 1:
                        break
                    addZ += 1
                elif multZ > maxMultZ:
                    break
            if not visitor((addX, addY, addZ), *args, **kwargs):
                break
        # non-exact versions of this algorithm might never reach the end voxel,
        # so we abort early using a different criterion
        if steppedX and multX > maxMultX:
            break
        if steppedY and multY > maxMultY:
            break
        if steppedZ and multZ > maxMultZ:
            break
        if not visitor((X, Y, Z), *args, **kwargs):
            break

def transform3(matrix, point):
    x_neu = point[0] * matrix[0] + point[1] * matrix[4] + point[2] * matrix[8]
    y_neu = point[0] * matrix[1] + point[1] * matrix[5] + point[2] * matrix[9]
    z_neu = point[0] * matrix[2] + point[1] * matrix[6] + point[2] * matrix[10]
    x_neu += matrix[12]
    y_neu += matrix[13]
    z_neu += matrix[14]
    return (x_neu, y_neu, z_neu)

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
    parser = argparse.ArgumentParser(description="Segment point cloud into static and dynamic points")
    parser.add_argument("-s", "--start", type=int, default=0, metavar="NUM")
    parser.add_argument("-e", "--end", type=int, default=-1, metavar="NUM")
    parser.add_argument("-f", "--format", type=formatname_to_io_type, default=py3dtk.IOType.UOS)
    parser.add_argument("--fuzz", type=float, default=0, help="How fuzzy the data is. I.e. how far points on a perfect plane are allowed to lie away from it in the scan (default: 0).")
    parser.add_argument("--voxel-size", type=float, default=10, metavar="SIZE",
        help="Voxel grid size (default: 10)")
    parser.add_argument("--diff", type=int, default=0, metavar="NUM",
        help="Number of scans before and after the current scan that are grouped together (default: 0).")
    parser.add_argument("--no-subvoxel-accuracy", action='store_true', help="Do not calculate with subvoxel accuracy")
    parser.add_argument("--min-cluster-size", type=int, default=5, help="Minimum number of connected voxels that make a dynamic object (default: 5). Set to any value greater than one to turn on clustering.")
    parser.add_argument(
        "--maxrange-method", choices=["none", "normals", "1nearest"], default="none",
        help="How to compute search range. Possible values: none, normals, 1nearest")
    parser.add_argument(
        "--write-maxranges", action='store_true', help="Write computed maxranges to scan002.3d and onward for each input scan")
    parser.add_argument("--normal-knearest", type=int, default=40, metavar="K",
        help="To compute the normal vector, use NUM closest points for --maxrange-method=normals (default: 40)")
    parser.add_argument("--normal-method", choices=["knearest", "range", "angle", "knearest-global", "range-global"], default="angle",
        help="How to select points to compute the normal from. Possible values: knearest (choose k using --normal-knearest), range (range search of voxel radius), angle (all points seen under the angle that one voxel is seen from the perspective of the scanner), knearest-global (like knearest but from a global k-d tree), rangle-global (like range but from a global k-d tree)")
    parser.add_argument("--maskdir", help="Directory to store .mask files. Default: ${directory}/pplremover")
    parser.add_argument("directory")
    args = parser.parse_args()

    voxel_size = args.voxel_size

    points_by_slice = dict()
    trajectory = dict()

    print("directory: %s" % args.directory, file=sys.stderr)
    print("voxel size: %f" % voxel_size, file=sys.stderr)
    print("reading data...", file=sys.stderr)

    voxel_diagonal = math.sqrt(3*voxel_size*voxel_size)

    scanserver = False
    py3dtk.openDirectory(scanserver, args.directory, args.format, args.start, args.end)

    len_trajectory = len(py3dtk.allScans)
    print("length: %d" % len_trajectory, file=sys.stderr)
    print("start: %d" % int(py3dtk.allScans[0].getIdentifier()), file=sys.stderr)
    print("end: %d" % int(py3dtk.allScans[-1].getIdentifier()), file=sys.stderr)

    scanorder = list()
    for s in py3dtk.allScans:
        i = int(s.getIdentifier())
        scanorder.append(i)
        print("%f" % ((((i-args.start)+1)*100)/len_trajectory), end="\r", file=sys.stderr)
        # ignore points that are closer than a voxel diagonal
        s.setRangeFilter(-1, voxel_diagonal)
        xyz_orig = list(py3dtk.DataXYZ(s.get("xyz")))
        # transform all points into the global coordinate system
        transmat = s.get_transMatOrg()
        s.transformAll(transmat)
        # add the current position to the trajectory
        trajectory[i] = (s.get_rPos(),s.get_rPosTheta(),transmat)
        # add all the points into their respective slices
        xyz = list(py3dtk.DataXYZ(s.get("xyz")))
        refl = py3dtk.DataReflectance(s.get("reflectance"))
        if len(refl) != 0:
            if len(xyz) != len(refl) or len(xyz_orig) != len(refl):
                print("unequal lengths %d %d in %d" % (len(xyz), len(refl), i))
                exit(1)
        else:
            refl = [ 0.0 ] * len(xyz)
        points = list(zip(xyz,xyz_orig,refl))
        points_by_slice[i] = points
        print("number of points in scan %d: %d" % (i, len(points)), file=sys.stderr)

    print("calculate voxel occupation", file=sys.stderr)

    voxel_occupied_by_slice = defaultdict(set)

    for i, points in points_by_slice.items():
        print("%f" % (((i+1)*100)/len_trajectory), end="\r", file=sys.stderr)
        for (x,y,z),_,_ in points:
            voxel_occupied_by_slice[voxel_of_point((x,y,z), voxel_size)].add(i)
    print("", file=sys.stderr)

    if len(voxel_occupied_by_slice) == 0:
        print("no voxel occupied", file=sys.stderr)
        exit(1)

    print("occupied voxels: %d" % len(voxel_occupied_by_slice), file=sys.stderr)

    print("compute maxranges", file=sys.stderr)

    free_voxels = set()

    maxranges = dict()
    for i in trajectory.keys():
        #maxranges.append([sq.length(p)-voxel_diagonal for (_,p,_) in points_by_slice[i]])
        maxranges[i] = [None]*len(points_by_slice[i])
    if args.maxrange_method == "normals":
        if args.normal_method in ["knearest-global", "range-global"]:
            # We build one big kd-tree because normal computation becomes harder
            # the further away from the scanner it is done (less points per volume)
            print("building global k-d tree", file=sys.stderr)
            kdtree = py3dtk.KDtree([p for points in points_by_slice.values() for  p,_,_ in points])
        for i, (pos,theta,transmat) in trajectory.items():
            # create a transmat to inverse the normal vector
            # this transmat must only contain a rotation part which is why we
            # cannot take the transmat of this scan
            #transmat = py3dtk.EulerToMatrix4((0,0,0), theta)
            #transmat = py3dtk.EulerToMatrix4(pos, theta)
            transmat4inv = py3dtk.M4inv(transmat)
            # build a quad tree
            print("building spherical quad tree", file=sys.stderr)
            qtree = sq.QuadTree([p for _,p,_ in points_by_slice[i]])
            # no need to build a k-d tree for the "angle" method
            if args.normal_method in ["knearest", "range"]:
                print("building local k-d tree", file=sys.stderr)
                kdtree = py3dtk.KDtree([p for _,p,_ in points_by_slice[i]])
            # for debugging purposes we create a dictionary associating every
            # point with the id of the one that it gets shadowed by
            point_shadowed_by = dict()
            print("calculating ranges")
            # Precompute the distances so that they are not computed multiple
            # times while sorting
            distances = { p:sq.length(p) for _,p,_ in points_by_slice[i] }
            # sort points by their distance from the scanner but keep their
            # original index to update the correct corresponding entry in
            # maxranges
            points = sorted(list(enumerate(points_by_slice[i])),
                    key=cmp_to_key(lambda a,b: distances[a[1][1]] - distances[b[1][1]]))
            # for each point in this scan (starting from the point closest to
            # the scanner) use its normal to calculate until when the line of
            # sight up to the point should be searched and apply the same limit
            # to all the points in its "shadow"
            for j,(p_global,p,_) in points:
                # point was already assigned a maximum range
                if maxranges[i][j] is not None:
                    continue
                p_norm = sq.norm(p)
                # FIXME: several ways to compute normal:
                #   - knearest (which k?)
                #   - fixed voxel diagonal radius search
                #   - all points within angular radius of voxel diagonal as seen from the scanner
                #   - only points closer to the scanner than the current point
                if args.normal_method == "knearest":
                    #k_nearest = [p_k for p_k in k_nearest if sq.length(p_k) < sq.length(p)]
                    normal, _ = py3dtk.calculateNormal(
                        kdtree.kNearestNeighbors(p, args.normal_knearest))
                elif args.normal_method == "range":
                    normal, _ = py3dtk.calculateNormal(
                        kdtree.fixedRangeSearch(p, (voxel_diagonal/2)**2))
                elif args.normal_method == "angle":
                    angle = 2*math.asin(voxel_diagonal/(sq.length(p)-voxel_diagonal))
                    normal, _ = py3dtk.calculateNormal(
                        [points_by_slice[i][k][1] for k in qtree.search(p_norm, angle)])
                elif args.normal_method == "knearest-global":
                    k_nearest = kdtree.kNearestNeighbors(p_global, args.normal_knearest)
                    # FIXME: instead of transforming the k_nearest points to
                    # the local coordinate system, figure out how to transform
                    # just the resulting normal vector
                    normal, _ = py3dtk.calculateNormal([py3dtk.transform3(transmat4inv, kn) for kn in k_nearest])
                    # make the normal orientation relative to the scanner
                    # coordinate system
                    #if not args.no_global_normals:
                    #    normal = py3dtk.transform3normal(transmat4inv, normal)
                elif args.normal_method == "range-global":
                    k_nearest = kdtree.fixedRangeSearch(p_global, (voxel_diagonal/2)**2)
                    # FIXME: same as for "knearest-global"
                    normal, _ = py3dtk.calculateNormal([py3dtk.transform3(transmat4inv, kn) for kn in k_nearest])
                else:
                    raise NotImplementedError("normal method not implemented: %s"%args.normal_method)
                # make sure that the normal vector points *toward* the scanner
                # we don't need to compute the acos to get the real angle
                # between the point vector and the normal because values less
                # than zero map to angles of more than 90 degrees
                angle_cos = normal[0]*p_norm[0]+normal[1]*p_norm[1]+normal[2]*p_norm[2]
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
                # normal vector is perpendicular to the line of sight up to the
                # point
                if divisor == 0:
                    maxranges[i][j] = 0
                    point_shadowed_by[j] = j
                    continue
                if dividend/divisor > sq.length(p):
                    raise Exception("p got lengthened: ", dividend/divisor)
                maxranges[i][j] = dividend/divisor
                point_shadowed_by[j] = j
                # the scanner itself is situated close to the plane that p
                # is part of. Thus, shoot no ray to this point at all.
                if maxranges[i][j] < 0:
                    # FIXME: should we "continue" here?
                    maxranges[i][j] = 0
                # points must not be too close or otherwise they will shadow
                # *all* the points
                if sq.length(p) < voxel_diagonal:
                    raise Exception("point too close to scanner")
                # now find all the points in the shadow of this one
                # the size of the shadow is determined by the angle under
                # which the voxel diagonal is seen at that distance
                #
                # We compute the angle under which the circumsphere of a voxel
                # is seen to compute the shadow. We consider the worst case
                # where the target point lies on the furthest point of the
                # circumsphere. Thus, the center of the circumsphere is at
                # the distance of the point minus its the radius of the
                # circumsphere.
                angle = 2*math.asin(voxel_diagonal/(sq.length(p)-voxel_diagonal))
                for k in qtree.search(p_norm, angle):
                    p_k = points_by_slice[i][k][1]
                    p_k_norm = sq.norm(p_k)
                    divisor = p_k_norm[0]*normal[0]+p_k_norm[1]*normal[1]+p_k_norm[2]*normal[2]
                    # normal vector is perpendicular to the line of sight up to
                    # the point
                    if divisor == 0:
                        continue
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
                    # FIXME: make sure that the current point is more than voxel_diagonal away from the plane
                    # FIXME: make sure that the maxrange for a point is not closer to it than voxel_diagonal
                    maxranges[i][k] = d
                    point_shadowed_by[k] = j
            shadowdir = os.path.join(args.directory, "pointshadows")
            if not os.path.exists(shadowdir):
                os.mkdir(shadowdir)
            (pos,theta,_) = trajectory[i]
            with open(os.path.join(shadowdir, "scan%03d.pose" % i), "w") as f:
                print("%f %f %f"%pos, file=f)
                print("%f %f %f"%tuple([t*180/math.pi for t in theta]), file=f)
            with open(os.path.join(shadowdir, "scan%03d.3d" % i), "w") as f:
                for j,(_,(x,y,z),_) in points:
                    random.seed(point_shadowed_by[j])
                    r=random.randint(0,255)
                    g=random.randint(0,255)
                    b=random.randint(0,255)
                    line = "%s %s %s %d %d %d\n"%(
                        py3dtk.utils.float2hex(x),
                        py3dtk.utils.float2hex(y),
                        py3dtk.utils.float2hex(z),
                        r, g, b)
                    f.write(line)
            del distances
            del points
            del qtree
            del point_shadowed_by
            if args.normal_method in ["knearest", "range"]:
                del kdtree
        if args.normal_method in ["knearest-global", "range-global"]:
            del kdtree
    elif args.maxrange_method == "1nearest":
        # this method is exact for single scans (because no normals can be
        # wrongly computed) but slower (kd-tree has to be searched for every
        # single point) and it fails for multiple scans (bremen-city effect)
        for i, (pos,theta,transmat) in trajectory.items():
            # we must not use global kd-trees because then the search would
            # stop at any obstacle and would never see "through" anything
            kdtree = py3dtk.KDtree([p for _,p,_ in points_by_slice[i]])
            for j,(_,p,_) in enumerate(points_by_slice[i]):
                maxranges[i][j] = sq.length(kdtree.segmentSearch_1NearestPoint((0,0,0), p, voxel_diagonal**2)) - voxel_diagonal
            del kdtree

    if args.write_maxranges and args.maxrange_method != "none":
        print("write maxranges", file=sys.stderr)
        for i, (pos,theta,_) in trajectory.items():
            with open("scan%03d.pose" % (i+2), "w") as f:
                print("%f %f %f"%pos, file=f)
                print("%f %f %f"%tuple([t*180/math.pi for t in theta]), file=f)
            with open("scan%03d.3d" % (i+2), "w") as f:
                #for j,(p_global,p,_) in enumerate(points_by_slice[i]):
                #    maxrange = maxranges[i][j]
                #    d = tuple(p)
                #    r = sq.length(d)
                #    factor = maxrange/r
                #    if factor > 1:
                #        raise Exception("duck 2: %f" % factor)
                #    if factor < 0:
                #        raise Exception("duck 3: ", factor, maxrange, r)
                #    d = d[0]*factor, d[1]*factor, d[2]*factor
                #    k_nearest = kdtree.kNearestNeighbors(p_global, 40)
                #    normal, _ = py3dtk.calculateNormal(k_nearest)
                #    p_norm = sq.norm(p)
                #    angle_cos = normal[0]*p_norm[0]+normal[1]*p_norm[1]+normal[2]*p_norm[2]
                #    #if math.acos(angle_cos) < 0.5*math.pi:
                #    if angle_cos < 0:
                #        # make sure that the normal turns toward the scanner
                #        normal = (-1*normal[0], -1*normal[1], -1*normal[2])
                #    print("%f %f %f %d %d %d" % (p+tuple([127.5+n*127.5 for n in normal])), file=f)
                for j,(_,p,_) in enumerate(points_by_slice[i]):
                    maxrange = maxranges[i][j]
                    d = tuple(p)
                    r = sq.length(d)
                    factor = maxrange/r
                    if factor > 1:
                        raise Exception("duck 2: %f" % factor)
                    if factor < 0:
                        raise Exception("duck 3: ", factor, maxrange, r)
                    d = d[0]*factor, d[1]*factor, d[2]*factor
                    print("%f %f %f 0" % d, file=f)

    print("walk voxels")
    for i, (pos,_,transmat) in trajectory.items():
        #print("%f" % (((i+1)*100)/len_trajectory), end="\r", file=sys.stderr)
        for j,(p,p_orig,_) in enumerate(points_by_slice[i]):
            #print("%f (%d)" % (((i+1)*100)/len_trajectory, j), end="\r", file=sys.stderr)
            if maxranges[i][j] is not None:
                maxrange = maxranges[i][j]
                r = sq.length(p_orig)
                factor = maxrange/r
                p = p_orig[0]*factor, p_orig[1]*factor, p_orig[2]*factor
                p = transform3(transmat, p)
            free = set()
            walk_voxels(pos, p, voxel_size, visitor, free, voxel_occupied_by_slice, i, args.diff)
            free_voxels |= free

    print("number of freed voxels: %d (%f %% of occupied voxels)" % (len(free_voxels), 100*len(free_voxels)/len(voxel_occupied_by_slice)), file=sys.stderr)

    if args.min_cluster_size > 1:
        print("clustering voxels")
        voxel_to_cluster = dict()
        cluster_to_voxel = dict()
        for i,voxel in enumerate(free_voxels):
            neighbor_clusters = set()
            for ix in range(-1, 2):
                for iy in range(-1, 2):
                    for iz in range(-1, 2):
                        neighbor_cluster = voxel_to_cluster.get((voxel[0]+ix, voxel[1]+iy, voxel[2]+iz))
                        if neighbor_cluster is not None:
                            neighbor_clusters.add(neighbor_cluster)
            # voxel has no clustered neighbours yet: start a new cluster
            if len(neighbor_clusters) == 0:
                voxel_to_cluster[voxel] = i
                cluster_to_voxel[i] = set([voxel])
                continue
            # just one neighboring cluster: add current voxel to it
            if len(neighbor_clusters) == 1:
                cluster = neighbor_clusters.pop()
                voxel_to_cluster[voxel] = cluster
                cluster_to_voxel[cluster].add(voxel)
                continue
            # more than one neighboring cluster: join clusters
            mincluster = min(neighbor_clusters)
            for cluster in neighbor_clusters:
                if cluster == mincluster:
                    continue
                for v in cluster_to_voxel[cluster]:
                    voxel_to_cluster[v] = mincluster
                cluster_to_voxel[mincluster].update(cluster_to_voxel[cluster])
                del cluster_to_voxel[cluster]
            voxel_to_cluster[voxel] = mincluster
            cluster_to_voxel[mincluster].add(voxel)
        for voxels in cluster_to_voxel.values():
            if len(voxels) < args.min_cluster_size:
                for voxel in voxels:
                    free_voxels.remove(voxel)
        print("number of free voxels after clustering: %d" % len(free_voxels))

    half_voxels = defaultdict(set)
    if not args.no_subvoxel_accuracy:
        print("calculate half-free voxels")
        for voxel in free_voxels:
            freed_slices = voxel_occupied_by_slice[voxel]
            neighbor_voxels = set()
            vradius = 2
            for ix in range(0-vradius, 1+vradius):
                for iy in range(0-vradius, 1+vradius):
                    for iz in range(0-vradius, 1+vradius):
                        neighbor_voxels.add((voxel[0]+ix, voxel[1]+iy, voxel[2]+iz))
            # remove the current voxel
            neighbor_voxels.remove(voxel)
            # remove voxels that were already marked as free from the neighbor
            neighbor_voxels -= free_voxels
            for neighbor in neighbor_voxels:
                # check if there is anything in the current neighbor
                if neighbor not in voxel_occupied_by_slice:
                    continue
                half_voxels[neighbor] |= freed_slices
        # make sure that no voxels are completely cleared
        #half_voxels = {v:i for v,i in half_voxels.items() if i != voxel_occupied_by_slice[v]}
        print("number of half-free voxels: %d"%len(half_voxels))

    print("write partitioning", file=sys.stderr)

    with open("scan000.3d", "w") as f1, open("scan001.3d", "w") as f2:
        for i in scanorder:
            points = points_by_slice[i]
            print("%f" % (((i+1)*100)/len_trajectory), end="\r", file=sys.stderr)
            for (x,y,z),_,r in points:
                voxel = voxel_of_point((x,y,z), voxel_size)
                line = "%s %s %s %s\n"%(
                    py3dtk.utils.float2hex(x),
                    py3dtk.utils.float2hex(y),
                    py3dtk.utils.float2hex(z),
                    py3dtk.utils.float2hex(r))
                if voxel in free_voxels or i in half_voxels.get(voxel, set()):
                    # this point is in a voxel marked as free or is a point
                    # from a slice index that was freed in an adjacent voxel
                    f2.write(line)
                else:
                    f1.write(line)
    for pose in ["scan000.pose", "scan001.pose"]:
        with open(pose, "w") as f:
            f.write("0 0 0\n0 0 0\n");

    print("write masks", file=sys.stderr)

    if args.maskdir is not None:
        maskdir = args.maskdir
    else:
        maskdir = os.path.join(args.directory, "pplremover")
    if not os.path.exists(maskdir):
        os.mkdir(maskdir)
    for i, points in points_by_slice.items():
        with open(os.path.join(maskdir, "scan%03d.mask" % i), "w") as f:
            print("%f" % (((i+1)*100)/len_trajectory), end="\r", file=sys.stderr)
            for (x,y,z),_,_ in points:
                voxel = voxel_of_point((x,y,z), voxel_size)
                if voxel in free_voxels or i in half_voxels.get(voxel, set()):
                    # this point is in a voxel marked as free or is a point
                    # from a slice index that was freed in an adjacent voxel
                    f.write("1\n")
                else:
                    f.write("0\n")
    #for frames in ["scan000.frames", "scan001.frames"]:
    #    with open(frames, "w") as f:
    #        f.write("1 0 0 0 0 1 0 0 0 0 1 0 0 0 0 1 2\n");
    print("", file=sys.stderr)
    print("done", file=sys.stderr)

if __name__ == "__main__":
    main()
