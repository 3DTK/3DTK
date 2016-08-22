#include <unordered_map>
#include <set>
#include <tuple>
#include <boost/functional/hash.hpp>
#include <limits>
#include <iostream>
#include <fstream>

#include "scanio/scan_io.h"
#include "slam6d/scan.h"

/*
 * define a hash function for 3-tuples
 */
namespace std
{
	template<> struct hash<std::tuple<ssize_t, ssize_t, ssize_t>>
	{
		std::size_t operator()(std::tuple<ssize_t, ssize_t, ssize_t> const& t) const
		{
			std::size_t seed = 0;
			boost::hash_combine(seed, std::get<0>(t));
			boost::hash_combine(seed, std::get<1>(t));
			boost::hash_combine(seed, std::get<2>(t));
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

void voxel_of_point(double x, double y, double z, double voxel_size, ssize_t *X, ssize_t *Y, ssize_t *Z)
{
	*X = py_div(x, voxel_size);
	*Y = py_div(y, voxel_size);
	*Z = py_div(z, voxel_size);
}

std::set<std::tuple<ssize_t,ssize_t,ssize_t>> walk_voxels(
		double *start,
		double *end,
		double voxel_size,
		std::unordered_map<std::tuple<ssize_t, ssize_t, ssize_t>, std::set<size_t>> const& voxel_occupied_by_slice,
		size_t current_slice,
		double max_search_distance,
		size_t diff,
		double max_target_dist,
		double max_target_proximity
		)
{
	double *direction = new double[3];
	direction[0] = end[0] - start[0];
	direction[1] = end[1] - start[1];
	direction[2] = end[2] - start[2];
	double dist = sqrt(direction[0]*direction[0]+direction[1]*direction[1]+direction[2]*direction[2]);
	if (max_target_dist != -1) {
		if (dist > max_target_dist) {
			return std::set<std::tuple<ssize_t,ssize_t,ssize_t>>();
		}
	}
	double tMax;
	if (max_search_distance == -1) {
		tMax = 1.0;
	} else {
		tMax = max_search_distance/dist;
		if (tMax > 1.0) {
			tMax = 1.0;
		}
	}
	if (max_target_proximity != -1) {
		tMax -= max_target_proximity/dist;
	}
	ssize_t X, Y, Z;
	voxel_of_point(start[0],start[1],start[2],voxel_size, &X, &Y, &Z);
	ssize_t startX = X, startY = Y, startZ = Z;
	ssize_t endX, endY, endZ;
	voxel_of_point(end[0],end[1],end[2],voxel_size, &endX, &endY, &endZ);
	double tDeltaX, tMaxX;
	double tDeltaY, tMaxY;
	double tDeltaZ, tMaxZ;
	char stepX, stepY, stepZ;
	if (direction[0] == 0) {
		tDeltaX = 0;
		stepX = 0;
		tMaxX = std::numeric_limits<double>::infinity();
	} else {
		stepX = direction[0] > 0 ? 1 : -1;
		tDeltaX = stepX*voxel_size/direction[0];
		tMaxX = tDeltaX * (1.0 - py_mod(stepX*(start[0]/voxel_size), 1.0));
	}
	if (direction[1] == 0) {
		tDeltaY = 0;
		stepY = 0;
		tMaxY = std::numeric_limits<double>::infinity();
	} else {
		stepY = direction[1] > 0 ? 1 : -1;
		tDeltaY = stepY*voxel_size/direction[1];
		tMaxY = tDeltaY * (1.0 - py_mod(stepY*(start[1]/voxel_size), 1.0));
	}
	if (direction[2] == 0) {
		tDeltaZ = 0;
		stepZ = 0;
		tMaxZ = std::numeric_limits<double>::infinity();
	} else {
		stepZ = direction[2] > 0 ? 1 : -1;
		tDeltaZ = stepZ*voxel_size/direction[2];
		tMaxZ = tDeltaZ * (1.0 - py_mod(stepZ*(start[2]/voxel_size), 1.0));
	}
	if (stepX == -1 && tMaxX == tDeltaX) {
		tMaxX = 0.0;
	}
	if (stepY == -1 && tMaxY == tDeltaY) {
		tMaxY = 0.0;
	}
	if (stepZ == -1 && tMaxZ == tDeltaZ) {
		tMaxZ = 0.0;
	}
	size_t multX = 0, multY = 0, multZ = 0;
	std::set<std::tuple<ssize_t,ssize_t,ssize_t>> empty_voxels;
	double epsilon = 1e-13;
	while (X != endX || Y != endY || Z != endZ) {
		if (tMaxX > 1.0+epsilon && tMaxY > 1.0+epsilon && tMaxZ > 1.0+epsilon) {
			std::cerr << "error 1" << start[0] << " " << start[1] << " " << start[2] << " " << end[0] << " " << end[1] << " " << end[2] << std::endl;
			break;
		}
		if (tMaxX > tMax-epsilon && tMaxY > tMax-epsilon && tMaxZ > tMax-epsilon) {
			break;
		}
		if (tMaxX < tMaxY) {
			if (tMaxX < tMaxZ) {
				multX += 1;
				X = startX + multX*stepX;
				tMaxX += tDeltaX;
			} else {
				multZ += 1;
				Z = startZ + multZ*stepZ;
				tMaxZ += tDeltaZ;
			}
		} else {
			if (tMaxY < tMaxZ) {
				multY += 1;
				Y = startY + multY*stepY;
				tMaxY += tDeltaY;
			} else {
				multZ += 1;
				Z = startZ + multZ*stepZ;
				tMaxZ += tDeltaZ;
			}
		}
		auto scanslices = voxel_occupied_by_slice.find(std::make_tuple(X,Y,Z));
		if (scanslices == voxel_occupied_by_slice.end()) {
			continue;
		}
		if (diff == 0) {
			if (scanslices->second.find(current_slice) == scanslices->second.end()) {
				empty_voxels.insert(std::make_tuple(X,Y,Z));
				continue;
			}
			break;
		} else {
			bool found = false;
			for (std::set<size_t>::iterator it = scanslices->second.begin(); it != scanslices->second.end(); ++it) {
				// it is important that we do not subtract values from *it or
				// current_slice because both are unsigned values and if they
				// are too small, then subtraction might underflow them
				if ((*it + diff >= current_slice) && (*it <= (current_slice + diff))) {
					found = true;
					break;
				}
			}
			if (found == false) {
				empty_voxels.insert(std::make_tuple(X,Y,Z));
				continue;
			} else {
				break;
			}
		}
	}
	delete[] direction;
	return empty_voxels;
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

	std::vector<std::vector<std::tuple<double,double,double,double>>> points_by_slice;
	std::vector<double*> trajectory;
	for(ScanVector::iterator scan = Scan::allScans.begin(); scan != Scan::allScans.end(); ++scan) {
		/* The range filter must be set *before* transformAll() because
		 * otherwise, transformAll will move the point coordinates such that
		 * the rangeFilter doesn't filter the right points anymore. This in
		 * turn can lead to the situation that when retrieving the reflectance
		 * values, some reflectance values are not returned because when
		 * reading them in, those get filtered by the *original* point
		 * coordinates. This in turn can lead to the situation that the vector
		 * for xyz and reflectance are of different length.
		 */
		(*scan)->setRangeFilter(-1, 10);
		(*scan)->transformAll((*scan)->get_transMatOrg());
		double *rPos = new double[3];
		rPos[0] = (*scan)->get_rPos()[0];
		rPos[1] = (*scan)->get_rPos()[1];
		rPos[2] = (*scan)->get_rPos()[2];
		trajectory.push_back(rPos);
		DataXYZ xyz((*scan)->get("xyz"));
		DataReflectance refl((*scan)->get("reflectance"));
		std::vector<std::tuple<double,double,double,double>> points;
		if (xyz.size() != refl.size()) {
			exit(1);
		}
		for (size_t i = 0; i < xyz.size(); ++i) {
			points.push_back(std::make_tuple(xyz[i][0],xyz[i][1],xyz[i][2],refl[i]));
		}
		points_by_slice.push_back(points);
	}

	std::unordered_map<std::tuple<ssize_t, ssize_t, ssize_t>, std::set<size_t>> voxel_occupied_by_slice;
	for (size_t i = 0; i < points_by_slice.size(); ++i) {
		for (size_t j = 0; j < points_by_slice[i].size(); ++j) {
			double x = std::get<0>(points_by_slice[i][j]);
			double y = std::get<1>(points_by_slice[i][j]);
			double z = std::get<2>(points_by_slice[i][j]);
			ssize_t X, Y, Z;
			voxel_of_point(x,y,z,voxel_size, &X, &Y, &Z);
			voxel_occupied_by_slice[std::make_tuple(X,Y,Z)].insert(i);
		}
	}

	std::set<std::tuple<ssize_t,ssize_t,ssize_t>> free_voxels;

	for (size_t i = 0; i < trajectory.size(); ++i) {
		for (size_t j = 0; j < points_by_slice[i].size(); ++j) {
			double *pointPos = new double[3];
			pointPos[0] = std::get<0>(points_by_slice[i][j]);
			pointPos[1] = std::get<1>(points_by_slice[i][j]);
			pointPos[2] = std::get<2>(points_by_slice[i][j]);
			std::set<std::tuple<ssize_t,ssize_t,ssize_t>> free = walk_voxels(
					trajectory[i],
					pointPos,
					voxel_size,
					voxel_occupied_by_slice,
					i,
					max_search_distance,
					diff,
					max_target_distance,
					max_target_proximity
					);
			for (std::set<std::tuple<ssize_t,ssize_t,ssize_t>>::iterator it = free.begin(); it != free.end(); ++it) {
				free_voxels.insert(*it);
			}
			delete[] pointPos;
		}
	}

	std::cout << free_voxels.size() << " " << voxel_occupied_by_slice.size() << std::endl;

	for (size_t i = 0; i < trajectory.size(); ++i) {
		delete[] trajectory[i];
	}

	/*
	 * we use a FILE object instead of an ofstream to write the data because
	 * it gives better performance (probably because it's buffered) and
	 * because of the availability of fprintf
	 */
	FILE *out_static = fopen("scan000.3d", "w");
	FILE *out_dynamic = fopen("scan001.3d", "w");
	for (size_t i = 0; i < points_by_slice.size(); ++i) {
		for (size_t j = 0; j < points_by_slice[i].size(); ++j) {
			double x = std::get<0>(points_by_slice[i][j]);
			double y = std::get<1>(points_by_slice[i][j]);
			double z = std::get<2>(points_by_slice[i][j]);
			double r = std::get<3>(points_by_slice[i][j]);
			ssize_t X, Y, Z;
			voxel_of_point(x,y,z,voxel_size, &X, &Y, &Z);
			FILE *out;
			if (free_voxels.find(std::make_tuple(X,Y,Z)) == free_voxels.end()) {
				out = out_static;
			} else {
				out = out_dynamic;
			}
			// we print the mantissa with 13 hexadecimal digits because the
			// mantissa for double precision is 52 bits long which is 6.5
			// bytes and thus 13 hexadecimal digits
			fprintf(out, "%.013a %.013a %.013a %.013a\n", x, y, z, r);
		}
	}
	fclose(out_static);
	fclose(out_dynamic);

	return 0;
}
