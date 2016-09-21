#include <unordered_map>
#include <set>
#include <boost/functional/hash.hpp>
#include <limits>
#include <iostream>
#include <fstream>

#include "scanio/scan_io.h"
#include "slam6d/scan.h"

#ifdef _OPENMP
#include <omp.h>
#endif

/*
 * instead of this struct we could also use an std::tuple which would be
 * equally fast and use the same amount of memory but I just don't like the
 * verbosity of the syntax and how to access members via std::get<>(v)
 */
struct voxel{
	ssize_t x;
	ssize_t y;
	ssize_t z;

	voxel(const ssize_t X, const ssize_t Y, const ssize_t Z)
		: x(X), y(Y), z(Z) {}

	voxel(const struct voxel &other)
		: x(other.x), y(other.y), z(other.z) {}

	bool operator<(const struct voxel& rhs) const
	{
		if (x != rhs.x) {
			return x < rhs.x;
		}
		if (y != rhs.y) {
			return y < rhs.y;
		}
		return z < rhs.z;
	}

	bool operator==(const struct voxel& rhs) const
	{
		return x == rhs.x && y == rhs.y && z == rhs.z;
	}

	bool operator!=(const struct voxel& rhs) const
	{
		return x != rhs.x || y != rhs.y || z != rhs.z;
	}
};

/*
 * define a hash function for 3-tuples
 */
namespace std
{
	template<> struct hash<struct voxel>
	{
		std::size_t operator()(struct voxel const& t) const
		{
			std::size_t seed = 0;
			boost::hash_combine(seed, t.x);
			boost::hash_combine(seed, t.y);
			boost::hash_combine(seed, t.z);
			return seed;
		}
	};
};

/*
 * Integer division and modulo in C/C++ suck. We were told that the % operator
 * is the modulo but it is actually not. It calculates the remainder and not
 * the modulo. Furthermore, the % operator is incapable of operating on
 * floating point values. Lastly, even integer division is screwed up because
 * it rounds into different directions depending on whether the result is
 * positive or negative...
 *
 * We thus implement the sane and mathematically more correct behaviour of
 * Python here.
 */
ssize_t py_div(double a, double b)
{
	ssize_t q = a/b;
	double r = fmod(a,b);
	if ((r != 0) && ((r < 0) != (b < 0))) {
		q -= 1;
	}
	return q;
}

double py_mod(double a, double b)
{
	double r = fmod(a, b);
	if (r!=0 && ((r<0) != (b<0))) {
		r += b;
	}
	return r;
}

struct voxel voxel_of_point(const double* p, double voxel_size)
{
	return voxel(
			py_div(p[0], voxel_size),
			py_div(p[1], voxel_size),
			py_div(p[2], voxel_size));
}

/*
 * walk voxels as described in
 *   Fast Voxel Traversal Algorithm for Ray Tracing
 *   by John Amanatides, Andrew Woo
 *   Eurographics ’87
 *   http://www.cs.yorku.ca/~amana/research/grid.pdf
 */
void walk_voxels(
		const double * const start_pos,
		const double * const end_pos,
		const double voxel_size,
		std::unordered_map<struct voxel, std::set<size_t>> const& voxel_occupied_by_slice,
		const size_t current_slice,
		const double max_search_distance,
		const size_t diff,
		const double max_target_dist,
		const double max_target_proximity,
		std::set<struct voxel> &result
		)
{
	double *direction = new double[3];
	direction[0] = end_pos[0] - start_pos[0];
	direction[1] = end_pos[1] - start_pos[1];
	direction[2] = end_pos[2] - start_pos[2];
	const double dist = sqrt(direction[0]*direction[0]+direction[1]*direction[1]+direction[2]*direction[2]);
	// check if the point is too far away
	if (max_target_dist != -1) {
		if (dist > max_target_dist) {
			return;
		}
	}
	double tMax;
	// compute the t value representing the desired search radius
	if (max_search_distance == -1) {
		tMax = 1.0;
	} else {
		tMax = max_search_distance/dist;
		if (tMax > 1.0) {
			tMax = 1.0;
		}
	}
	// optionally subtract the set target proximity
	if (max_target_proximity != -1) {
		tMax -= max_target_proximity/dist;
	}
	const struct voxel start_voxel = voxel_of_point(start_pos,voxel_size);
	struct voxel cur_voxel = voxel(start_voxel);
	const struct voxel end_voxel = voxel_of_point(end_pos,voxel_size);
	double tDeltaX, tMaxX;
	double tDeltaY, tMaxY;
	double tDeltaZ, tMaxZ;
	char stepX, stepY, stepZ;
	/*
	 * tMax*: value t at which the segment crosses the first voxel boundary in the given direction
	 * stepX: in which direction to increase the voxel count (1 or -1)
	 * tDelta*: value t needed to span the voxel size in the given direction up to the target
	 */
	if (direction[0] == 0) {
		tDeltaX = 0;
		stepX = 0;
		tMaxX = std::numeric_limits<double>::infinity();
	} else {
		stepX = direction[0] > 0 ? 1 : -1;
		tDeltaX = stepX*voxel_size/direction[0];
		tMaxX = tDeltaX * (1.0 - py_mod(stepX*(start_pos[0]/voxel_size), 1.0));
	}
	if (direction[1] == 0) {
		tDeltaY = 0;
		stepY = 0;
		tMaxY = std::numeric_limits<double>::infinity();
	} else {
		stepY = direction[1] > 0 ? 1 : -1;
		tDeltaY = stepY*voxel_size/direction[1];
		tMaxY = tDeltaY * (1.0 - py_mod(stepY*(start_pos[1]/voxel_size), 1.0));
	}
	if (direction[2] == 0) {
		tDeltaZ = 0;
		stepZ = 0;
		tMaxZ = std::numeric_limits<double>::infinity();
	} else {
		stepZ = direction[2] > 0 ? 1 : -1;
		tDeltaZ = stepZ*voxel_size/direction[2];
		tMaxZ = tDeltaZ * (1.0 - py_mod(stepZ*(start_pos[2]/voxel_size), 1.0));
	}
	/*
	 * if we step into negative direction and the starting point happened to
	 * fall exactly onto the voxel boundary, then we need to reset the
	 * respective tMax value as we'd otherwise skip the first voxel in that
	 * direction
	 */
	if (stepX == -1 && tMaxX == tDeltaX) {
		tMaxX = 0.0;
	}
	if (stepY == -1 && tMaxY == tDeltaY) {
		tMaxY = 0.0;
	}
	if (stepZ == -1 && tMaxZ == tDeltaZ) {
		tMaxZ = 0.0;
	}
	/*
	 * in contrast to the original algorithm by John Amanatides and Andrew Woo
	 * we increment a counter and multiply the step size instead of adding up
	 * the steps. Doing the latter might introduce errors because due to
	 * floating point precision errors, 0.1+0.1+0.1 is unequal 3*0.1.
	 */
	size_t multX = 0, multY = 0, multZ = 0;
	const double epsilon = 1e-13;
	/*
	 * iterate until either:
	 *  - the final voxel is reached
	 *  - tMax is reached by all tMax-coordinates
	 *  - the current voxel contains points of (or around) the current scan
	 */
	while (cur_voxel != end_voxel) {
		// if we implemented the algorithm correctly, the following should
		// never happen, thus commenting it out
		/*
		if (tMaxX > 1.0+epsilon && tMaxY > 1.0+epsilon && tMaxZ > 1.0+epsilon) {
			std::cerr << "error 1" << start_pos[0] << " " << start_pos[1] << " " << start_pos[2] << " " << end_pos[0] << " " << end_pos[1] << " " << end_pos[2] << std::endl;
			exit(1)
		}
		*/
		/*
		 * We need an epsilon to support cases in which we end up searching up
		 * to the target voxel (with tMax = 1.0) and the target coordinate
		 * being exactly on the voxel boundary.
		 */
		if (tMaxX > tMax-epsilon && tMaxY > tMax-epsilon && tMaxZ > tMax-epsilon) {
			break;
		}
		if (tMaxX < tMaxY) {
			if (tMaxX < tMaxZ) {
				multX += 1;
				cur_voxel.x = start_voxel.x + multX*stepX;
				tMaxX += tDeltaX;
			} else {
				multZ += 1;
				cur_voxel.z = start_voxel.z + multZ*stepZ;
				tMaxZ += tDeltaZ;
			}
		} else {
			if (tMaxY < tMaxZ) {
				multY += 1;
				cur_voxel.y = start_voxel.y + multY*stepY;
				tMaxY += tDeltaY;
			} else {
				multZ += 1;
				cur_voxel.z = start_voxel.z + multZ*stepZ;
				tMaxZ += tDeltaZ;
			}
		}
		std::unordered_map<struct voxel, std::set<size_t>>::const_iterator scanslices_it =
			voxel_occupied_by_slice.find(cur_voxel);
		// if voxel has no points at all, continue searching without marking
		// the current voxel as free as it had no points to begin with
		if (scanslices_it == voxel_occupied_by_slice.end()) {
			continue;
		}
		std::set<size_t> scanslices = scanslices_it->second;
		/*
		 * The following implements a sliding window within which voxels
		 * that also contain points with a similar index as the current
		 * slice are not marked as free. Instead, the search aborts
		 * early.
		 * If no points around the current slice are found in the voxel,
		 * then the points in it only were seen from a very different scanner
		 * position and thus, these points are actually not there and
		 * the voxel must be marked as free.
		 *
		 * if the voxel has a point in it, only abort if the slice number
		 * is close to the current slice
		 */
		if (diff == 0) {
			// if the voxel contains the current slice, abort the search
			if (scanslices.find(current_slice) != scanslices.end()) {
				break;
			}
		} else {
			size_t window_start = current_slice - diff;
			// subtracting diff from an unsigned value might underflow it,
			// this check resets the window start to zero in that case
			if (diff > current_slice) {
				window_start = 0;
			}
			// first element that is equivalent or goes after value
			std::set<size_t>::iterator lower_bound = scanslices.lower_bound(window_start);
			// if elements in the set are found around the neighborhood of the
			// current slice, abort the search
			if (lower_bound != scanslices.end() && *lower_bound <= current_slice + diff) {
				break;
			}
		}
		result.insert(cur_voxel);
	}
	delete[] direction;
	return;
}

int main(int argc, char* argv[])
{
	size_t start = atoi(argv[1]);
	size_t end = atoi(argv[2]);
	char *dir = argv[3];

	double voxel_size = 5;
	double max_search_distance = 250;
	size_t diff = 285;
	double max_target_distance = 1000;
	double max_target_proximity = 30;

	Scan::openDirectory(false, dir, UOSR, start, end);
	if(Scan::allScans.size() == 0) {
		std::cerr << "No scans found. Did you use the correct format?" << std::endl;
		exit(-1);
	}

	std::vector<std::vector<double*>> points_by_slice;
	std::vector<std::vector<double>> reflectances_by_slice;
	std::vector<const double*> trajectory;
	std::unordered_map<struct voxel, std::set<size_t>> voxel_occupied_by_slice;
	for(size_t i = 0; i < Scan::allScans.size(); ++i) {
		Scan* scan = Scan::allScans[i];
		/* The range filter must be set *before* transformAll() because
		 * otherwise, transformAll will move the point coordinates such that
		 * the rangeFilter doesn't filter the right points anymore. This in
		 * turn can lead to the situation that when retrieving the reflectance
		 * values, some reflectance values are not returned because when
		 * reading them in, those get filtered by the *original* point
		 * coordinates. This in turn can lead to the situation that the vector
		 * for xyz and reflectance are of different length.
		 */
		scan->setRangeFilter(-1, 10);
		scan->transformAll(scan->get_transMatOrg());
		trajectory.push_back(scan->get_rPos());
		DataXYZ xyz(scan->get("xyz"));
		DataReflectance refl(scan->get("reflectance"));
		std::vector<double*> points;
		std::vector<double> reflectances;
		if (xyz.size() != refl.size()) {
			exit(1);
		}
		for (size_t j = 0; j < xyz.size(); ++j) {
			points.push_back(xyz[j]);
			reflectances.push_back(refl[j]);
			voxel_occupied_by_slice[voxel_of_point(xyz[j],voxel_size)].insert(i);
		}
		points_by_slice.push_back(points);
		reflectances_by_slice.push_back(reflectances);
	}

	std::set<struct voxel> free_voxels;

	struct timespec before, after;
	clock_gettime(CLOCK_MONOTONIC, &before);
#ifdef _OPENMP
	omp_set_num_threads(OPENMP_NUM_THREADS);
#pragma omp parallel for schedule(dynamic)
#endif
	for (size_t i = 0; i < trajectory.size(); ++i) {
		std::set<struct voxel> free;
		for (size_t j = 0; j < points_by_slice[i].size(); ++j) {
			walk_voxels(
					trajectory[i],
					points_by_slice[i][j],
					voxel_size,
					voxel_occupied_by_slice,
					i,
					max_search_distance,
					diff,
					max_target_distance,
					max_target_proximity,
					free
					);
		}
#ifdef _OPENMP
#pragma omp critical
#endif
		for (std::set<struct voxel>::iterator it = free.begin(); it != free.end(); ++it) {
			free_voxels.insert(*it);
		}
	}
	clock_gettime(CLOCK_MONOTONIC, &after);
	double elapsed = (after.tv_sec - before.tv_sec);
	elapsed += (after.tv_nsec - before.tv_nsec) / 1000000000.0;
	std::cerr << "took: " << elapsed << " seconds" << std::endl;

	std::cout << free_voxels.size() << " " << voxel_occupied_by_slice.size() << std::endl;

	/*
	 * we use a FILE object instead of an ofstream to write the data because
	 * it gives better performance (probably because it's buffered) and
	 * because of the availability of fprintf
	 */
	FILE *out_static = fopen("scan000.3d", "w");
	FILE *out_dynamic = fopen("scan001.3d", "w");
	for (size_t i = 0; i < points_by_slice.size(); ++i) {
		for (size_t j = 0; j < points_by_slice[i].size(); ++j) {
			FILE *out;
			if (free_voxels.find(voxel_of_point(points_by_slice[i][j],voxel_size)) == free_voxels.end()) {
				out = out_static;
			} else {
				out = out_dynamic;
			}
			// we print the mantissa with 13 hexadecimal digits because the
			// mantissa for double precision is 52 bits long which is 6.5
			// bytes and thus 13 hexadecimal digits
			fprintf(out, "%.013a %.013a %.013a %.013a\n",
					points_by_slice[i][j][0],
					points_by_slice[i][j][1],
					points_by_slice[i][j][2],
					reflectances_by_slice[i][j]
					);
		}
	}
	fclose(out_static);
	fclose(out_dynamic);

	return 0;
}
