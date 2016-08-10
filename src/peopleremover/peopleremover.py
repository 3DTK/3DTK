#!/usr/bin/python3

from collections import defaultdict
import sys
import math
import argparse
from multiprocessing import Pool
import os

sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..', 'lib'))
try:
	import py3dtk
except ImportError:
	print("Cannot find py3dtk module. Try recompiling 3dtk with WITH_PYTHON set to ON", file=sys.stderr)
	exit(1)

def iter_baskets_from(items, maxbaskets):
    '''generates evenly balanced baskets from indexable iterable'''
    item_count = len(items)
    baskets = min(item_count, maxbaskets)
    for x_i in range(baskets):
        yield [items[y_i] for y_i in range(x_i, item_count, baskets)]

def voxel_of_point(point, voxel_size):
	return int(point[0]//voxel_size), int(point[1]//voxel_size), int(point[2]//voxel_size)

def mp_init(vs, d, occ):
	global voxel_size
	global max_search_distance
	global voxel_occupied_by_slice
	voxel_size = vs
	max_search_distance = d
	voxel_occupied_by_slice = occ

def mp_proc(l):
	res = set()
	ll = len(l)
	j = 0
	for i,pos,p in l:
		print("%f" % (((j+1)*100)/ll), end="\r", file=sys.stderr)
		j+=1
		free = walk_voxels(pos, p, voxel_size, voxel_occupied_by_slice, i, max_search_distance)
		res |= free
	return res

# walk voxels as described in
#   Fast Voxel Traversal Algorithm for Ray Tracing
#   by John Amanatides, Andrew Woo
#   Eurographics ’87
#   http://www.cs.yorku.ca/~amana/research/grid.pdf
def walk_voxels(start, end, voxel_size, voxel_occupied_by_slice, current_slice, max_search_distance):
	#print("from: %f %f %f" % start)
	#print("to: %f %f %f" % end)
	X, Y, Z = voxel_of_point(start, voxel_size)
	endX, endY, endZ = voxel_of_point(end, voxel_size)
	#print("start: %d %d %d" % (X, Y, Z))
	#print("end: %d %d %d" % (endX, endY, endZ))
	direction = (end[0] - start[0], end[1] - start[1], end[2] - start[2])
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
	empty_voxels = set()
	#i = 0
	# compute the t value representing the desired search radius
	if max_search_distance == -1:
		tMax = 0.9
	else:
		tMax = max_search_distance/(voxel_size*math.sqrt(direction[0]*direction[0]+direction[1]*direction[1]+direction[2]*direction[2]))
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
				X += stepX
				tMaxX += tDeltaX
			else:
				Z += stepZ
				tMaxZ += tDeltaZ
		else:
			if tMaxY < tMaxZ:
				Y += stepY
				tMaxY += tDeltaY
			else:
				Z += stepZ
				tMaxZ += tDeltaZ
		#print("%f %f %f" % (tMaxX, tMaxY, tMaxZ))
		# if the voxel has no point in it at all, continue searching
		if (X, Y, Z) not in voxel_occupied_by_slice:
			# we don't need to add this voxel to the empty voxel list because it was
			# empty to begin with
			continue
		# if the voxel has a point in it, only abort if the slice number
		# is close to the current slice (use difference of 10 slices)
		#diff = 285 # full circle: 570
		#diff = 140
		diff = 0
		if diff == 0:
			if current_slice not in voxel_occupied_by_slice[(X, Y, Z)]:
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
#	x_neu = point[0] * matrix[0] + point[1] * matrix[4] + point[2] * matrix[8]
#	y_neu = point[0] * matrix[1] + point[1] * matrix[5] + point[2] * matrix[9]
#	z_neu = point[0] * matrix[2] + point[1] * matrix[6] + point[2] * matrix[10]
#	x_neu += matrix[12]
#	y_neu += matrix[13]
#	z_neu += matrix[14]
#	return (x_neu, y_neu, z_neu)

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

def main():
	parser = argparse.ArgumentParser()
	parser.add_argument("-s", "--start", type=int)
	parser.add_argument("-e", "--end", type=int)
	parser.add_argument("-f", "--format", type=formatname_to_io_type)
	parser.add_argument("--max-search-distance", type=int)
	parser.add_argument("-j", "--jobs", type=int, default=1)
	parser.add_argument("directory")
	args = parser.parse_args()

	voxel_size = 10

	points_by_slice = list()
	trajectory = list()

	len_trajectory = args.end - args.start + 1
	print("directory: %s" % args.directory, file=sys.stderr)
	print("start: %d" % args.start, file=sys.stderr)
	print("end: %d" % args.end, file=sys.stderr)
	print("length: %d" % len_trajectory, file=sys.stderr)
	print("voxel size: %f" % voxel_size, file=sys.stderr)
	print("reading data...", file=sys.stderr)

	scanserver = False
	py3dtk.openDirectory(scanserver, args.directory, args.format, args.start, args.end)
	for i,s in enumerate(py3dtk.allScans, start=args.start):
		print("%f" % (((i+1)*100)/len_trajectory), end="\r", file=sys.stderr)
		# transform all points into the global coordinate system
		s.transformAll(s.get_transMatOrg())
		# ignore points that are closer than 10 units
		s.setRangeFilter(-1, 10)
		# add the current position to the trajectory
		trajectory.append(s.get_rPos())
		# add all the points into their respective slices
		points = set()
		xyz = py3dtk.DataXYZ(s.get("xyz"))
		refl = py3dtk.DataReflectance(s.get("reflectance"))
		for (x,y,z),r in zip(xyz,refl):
			points.add((x,y,z,r))
		points_by_slice.append(points)
		print("number of points in scan %d: %d" % (i, len(points)), file=sys.stderr)

	print("calculate voxel occupation", file=sys.stderr)

	voxel_occupied_by_slice = defaultdict(set)

	for i, points in enumerate(points_by_slice):
		print("%f" % (((i+1)*100)/len_trajectory), end="\r", file=sys.stderr)
		for x,y,z,_ in points:
			voxel_occupied_by_slice[voxel_of_point((x,y,z), voxel_size)].add(i)
	print("", file=sys.stderr)

	if len(voxel_occupied_by_slice) == 0:
		print("no voxel occupied", file=sys.stderr)
		exit(1)

	print("occupied voxels: %d" % len(voxel_occupied_by_slice), file=sys.stderr)

	print("find free voxels", file=sys.stderr)

	free_voxels = set()

	# if requested, split the work into jobs
	if args.jobs == 1:
		for i, pos in enumerate(trajectory):
			print("%f" % (((i+1)*100)/len_trajectory), end="\r", file=sys.stderr)
			for j,(x,y,z,_) in enumerate(points_by_slice[i]):
				print("%f (%d)" % (((i+1)*100)/len_trajectory, j), end="\r", file=sys.stderr)
				free = walk_voxels(pos, (x,y,z), voxel_size, voxel_occupied_by_slice, i, args.max_search_distance)
				free_voxels |= free
	else:
		jobs = []
		for i,pos in enumerate(trajectory):
			for p in points_by_slice[i]:
				jobs.append((i,pos,p))
		# split list into as many parts as there are jobs
		chunks = iter_baskets_from(jobs, args.jobs)
		with Pool(args.jobs, mp_init, (voxel_size, args.max_search_distance, voxel_occupied_by_slice)) as p:
			res = p.map(mp_proc, chunks)
		free_voxels = set().union(*res)
		
	print("", file=sys.stderr)

	print("number of freed voxels: %d (%f %%)" % (len(free_voxels), 100*len(free_voxels)/len(voxel_occupied_by_slice)), file=sys.stderr)

	print("write result", file=sys.stderr)

	with open("scan000.3d", "w") as f1, open("scan001.3d", "w") as f2:
		for i, points in enumerate(points_by_slice):
			print("%f" % (((i+1)*100)/len_trajectory), end="\r", file=sys.stderr)
			for x,y,z,r in points:
				voxel = voxel_of_point((x,y,z), voxel_size)
				if voxel not in free_voxels:
					f1.write("%f %f %f %d\n" % (x,y,z,r))
				else:
					f2.write("%f %f %f %d\n" % (x,y,z,r))
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
