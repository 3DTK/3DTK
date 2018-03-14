#ifndef PEOPLEREMOVER_COMMON_H
#define PEOPLEREMOVER_COMMON_H

#include <unordered_map>
#include <set>
#include <boost/functional/hash.hpp>
#include <boost/program_options.hpp>
#include <limits>
#include <iostream>
#include <fstream>
#include <sys/stat.h>
#include <sys/types.h>

namespace po = boost::program_options;

#include "scanio/scan_io.h"
#include "slam6d/scan.h"
#include "slam6d/normals.h"
#include "spherical_quadtree/spherical_quadtree.h"

#ifdef _OPENMP
#include <omp.h>
#endif

#ifdef _MSC_VER
#include <BaseTsd.h>
typedef SSIZE_T ssize_t;
#endif

#include <boost/iostreams/device/mapped_file.hpp>

namespace io = boost::iostreams;

/*
 * instead of this struct we could also use an std::tuple which would be
 * equally fast and use the same amount of memory but I just don't like the
 * verbosity of the syntax and how to access members via std::get<>(v)
 */
struct voxel{
	mutable ssize_t x;
	mutable ssize_t y;
	mutable ssize_t z;

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

namespace std
{
	template<> struct hash<struct voxel>
	{
		std::size_t operator()(struct voxel const& t) const;
	};
};

ssize_t py_div(double a, double b);

double py_mod(double a, double b);

struct voxel voxel_of_point(const double *p, double voxel_size);

// FIXME: test the runtime if empty_voxels and voxel_occupied_by_slice are not
// pointers
struct visitor_args
{
	std::set<struct voxel> *empty_voxels;
	std::unordered_map<struct voxel, std::set<size_t>> *voxel_occupied_by_slice;
	size_t current_slice;
	size_t diff;
};

bool visitor(struct voxel voxel, void *data);

void walk_voxels(
		const double * const start_pos,
		const double * const end_pos,
		const double voxel_size,
		bool (*visitor)(struct voxel, void *),
		void *data
		);

void validate(boost::any& v, const std::vector<std::string>& values, IOType* target_type, int);

enum maxrange_method_t {NONE, NORMALS, ONENEAREST};

void validate(boost::any& v, const std::vector<std::string>& values, maxrange_method_t* target_type, int);

enum normal_method_t {KNEAREST, RANGE, ANGLE, KNEAREST_GLOBAL, RANGE_GLOBAL};

void validate(boost::any& v, const std::vector<std::string>& values, normal_method_t* target_type, int);

void parse_cmdline(int argc, char* argv[], ssize_t &start, ssize_t &end, IOType &format, double &fuzz,
	double &voxel_size, size_t &diff, size_t &normal_knearest,
	size_t &cluster_size, normal_method_t &normal_method,
	maxrange_method_t &maxrange_method, std::string &maskdir,
	std::string &staticdir, std::string &dir, bool &no_subvoxel_accuracy,
	bool &write_maxranges, int &jobs
#ifdef WITH_MMAP_SCAN
	, std::string &cachedir
#endif
	);

void compute_maxranges(std::vector<double> &maxranges, const DataXYZ &orig_points_by_slice,
				const normal_method_t normal_method,
		const double voxel_diagonal,
		const double fuzz
		);

#endif
