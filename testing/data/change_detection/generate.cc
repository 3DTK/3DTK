// to have M_PI defined on Windows
#ifdef _WIN32
#define _USE_MATH_DEFINES
#endif
#include <cmath>
#include <tuple>
#include <vector>
#include <limits>
#include <cstdio>
#include <sstream>
#include <iomanip>

#if defined(__CYGWIN__) || defined(__MINGW32__)
# ifndef M_PI
#  define M_PI       3.14159265358979323846
#  define M_PI_2     1.57079632679489661923
#  define M_PI_4     0.78539816339744830962
#  define M_1_PI     0.31830988618379067154
#  define M_2_PI     0.63661977236758134308
#  define M_SQRT2    1.41421356237309504880
#  define M_SQRT1_2  0.70710678118654752440
# endif
#endif // __CYGWIN__

enum Plane { XY, XZ, YZ };

double common(
		double o_x, double o_y, double o_z,
		double d_x, double d_y, double d_z,
		double z,
		double min_x, double max_x,
		double min_y, double max_y,
		double &out_x, double &out_y)
{
	if (d_z == 0) {
		return std::numeric_limits<double>::infinity();
	}
	double r_inters = (z - o_z)/d_z;
	if (r_inters < 0) {
		return std::numeric_limits<double>::infinity();
	}
	double x_inters = r_inters*d_x + o_x;
	if (min_x > x_inters || x_inters > max_x) {
		return std::numeric_limits<double>::infinity();
	}
	double y_inters = r_inters*d_y + o_y;
	if (min_y > y_inters || y_inters > max_y) {
		return std::numeric_limits<double>::infinity();
	}
	out_x = x_inters;
	out_y = y_inters;
	return r_inters;
}

double intersect_plane(
		double o_x, double o_y, double o_z,
		double d_x, double d_y, double d_z,
		std::tuple<Plane, double, double, double, double, double> r,
		double &out_x, double &out_y, double &out_z) {
	Plane p;
	double z, min_x, max_x, min_y, max_y;
	std::tie (p, z, min_x, max_x, min_y, max_y) = r;
	switch (p) {
		case XY:
			out_z = z;
			return common(o_x, o_y, o_z, d_x, d_y, d_z, z, min_x, max_x, min_y, max_y, out_x, out_y);
		case XZ:
			out_y = z;
			return common(o_x, o_z, o_y, d_x, d_z, d_y, z, min_x, max_x, min_y, max_y, out_x, out_z);
		case YZ:
			out_x = z;
			return common(o_y, o_z, o_x, d_y, d_z, d_x, z, min_x, max_x, min_y, max_y, out_y, out_z);
	}
}

int main()
{
	std::vector<std::tuple<Plane, double, double, double, double, double>> env = {
		std::make_tuple( YZ, 10, 0, 10, 0, 10 ),
		std::make_tuple( YZ, 0, 0, 10, 0, 10 ),
		std::make_tuple( XZ, 10, 0, 10, 0, 10 ),
		std::make_tuple( XZ, 0, 0, 10, 0, 10 ),
		std::make_tuple( XY, 0, 0, 10, 0, 10 )
	};
	std::vector<std::tuple<Plane, double, double, double, double, double>> box = {
		std::make_tuple( YZ, 6, 4, 6, 0, 2 ),
		std::make_tuple( YZ, 4, 4, 6, 0, 2 ),
		std::make_tuple( XZ, 6, 4, 6, 0, 2 ),
		std::make_tuple( XZ, 4, 4, 6, 0, 2 ),
		std::make_tuple( XY, 2, 4, 6, 4, 6 )
	};
	std::vector<std::tuple<double, double, double>> scanner_pos = {
		std::make_tuple(2,2,1),
		std::make_tuple(8,2,4),
		std::make_tuple(5,5,5),
		std::make_tuple(2,5,3),
		std::make_tuple(5,8,1),
	};
	int scan_num = 0;
	for (auto o: scanner_pos) {
		double o_x, o_y, o_z;
		std::tie (o_x, o_y, o_z) = o;
		std::stringstream ss;
		ss << "scan" << std::setfill('0') << std::setw(3) << scan_num << ".pose";
		FILE* fnobox_pose = std::fopen(ss.str().c_str(), "wb");
		std::fprintf(fnobox_pose, "%f %f %f\n0 0 0\n", -o_y, o_z, o_x);
		std::fclose(fnobox_pose);
		ss.str("");
		ss << "scan" << std::setfill('0') << std::setw(3) << scan_num << ".3d";
		FILE* fnobox = std::fopen(ss.str().c_str(), "wb");
		scan_num += 1;
		ss.str("");
		ss << "scan" << std::setfill('0') << std::setw(3) << scan_num << ".pose";
		FILE* fbox_pose = std::fopen(ss.str().c_str(), "wb");
		std::fprintf(fbox_pose, "%f %f %f\n0 0 0\n", -o_y, o_z, o_x);
		std::fclose(fbox_pose);
		ss.str("");
		ss << "scan" << std::setfill('0') << std::setw(3) << scan_num << ".3d";
		FILE* fbox = std::fopen(ss.str().c_str(), "wb");
		scan_num += 1;
		for (int incl = 1; incl < 180; ++incl) {
			double theta = incl/180.0*M_PI;
			double sint = sin(theta);
			double cost = cos(theta);
			double d_z = cost;
			for (int azim = 0; azim < 360; ++azim) {
				double phi = azim/360.0*M_PI*2;
				double sinp = sin(phi);
				double cosp = cos(phi);
				double d_x = sint*cosp;
				double d_y = sint*sinp;
				double mindist = std::numeric_limits<double>::infinity();
				double minx, miny, minz;
				for (auto p: env) {
					double x, y, z;
					double dist = intersect_plane(o_x, o_y, o_z, d_x, d_y, d_z, p, x, y, z);
					if (dist < mindist) {
						mindist = dist;
						minx = x;
						miny = y;
						minz = z;
					}
				}
				/*
				 * Do not print the values as exact hexfloats because depending
				 * on the glibc implementation of the trigonometric functions,
				 * the last digit might be rounded differently. Compare for
				 * example 2.27 with 2.28.
				 *
				 * We remove another digit because cygwin suffers from yet
				 * another sort of imprecision...
				 *
				 * We remove another digit of precision to make MacOS happy...
				 */
				if (mindist != std::numeric_limits<double>::infinity()) {
					std::fprintf(fnobox, "%.10f %.10f %.10f 0\n", -miny+o_y, minz-o_z, minx-o_x);
				}
				for (auto p: box) {
					double x, y, z;
					double dist = intersect_plane(o_x, o_y, o_z, d_x, d_y, d_z, p, x, y, z);
					if (dist < mindist) {
						mindist = dist;
						minx = x;
						miny = y;
						minz = z;
					}
				}
				if (mindist != std::numeric_limits<double>::infinity()) {
					std::fprintf(fbox, "%.10f %.10f %.10f 0\n", -miny+o_y, minz-o_z, minx-o_x);
				}
			}
		}
		std::fclose(fnobox);
		std::fclose(fbox);
	}
}
