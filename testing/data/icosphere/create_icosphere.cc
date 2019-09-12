#include <iostream>
#include <cmath>
#include <tuple>
#include <vector>
#include <fstream>
#include <string>

#ifdef _WIN32
#include <direct.h>
#define mkdir(path,mode) _mkdir (path)
#else
#include <sys/stat.h>
#include <sys/types.h>
#endif

std::tuple<double, double, double> normalize(double x, double y, double z)
{
	double length = sqrt(x*x+y*y+z*z);
	return std::make_tuple(x/length, y/length, z/length);
}

std::tuple<double, double, double> get_middle_point(std::tuple<double, double, double> p1, std::tuple<double, double, double> p2)
{
	return normalize(
			(std::get<0>(p1)+std::get<0>(p2))/2.0,
			(std::get<1>(p1)+std::get<1>(p2))/2.0,
			(std::get<2>(p1)+std::get<2>(p2))/2.0);
}

int main(int argc, char* argv[])
{
	// create 12 vertices of an icosahedron
	double t = (1.0 + sqrt(5.0))/2.0;
	std::vector<std::tuple<double, double, double>> points = {
		normalize(-1,  t,  0),
		normalize( 1,  t,  0),
		normalize(-1, -t,  0),
		normalize( 1, -t,  0),
		normalize( 0, -1,  t),
		normalize( 0,  1,  t),
		normalize( 0, -1, -t),
		normalize( 0,  1, -t),
		normalize( t,  0, -1),
		normalize( t,  0,  1),
		normalize(-t,  0, -1),
		normalize(-t,  0,  1)
	};

	std::vector<std::tuple<size_t, size_t, size_t>> faces = {
		std::make_tuple(0, 11, 5),
		std::make_tuple(0, 5, 1),
		std::make_tuple(0, 1, 7),
		std::make_tuple(0, 7, 10),
		std::make_tuple(0, 10, 11),
		std::make_tuple(1, 5, 9),
		std::make_tuple(5, 11, 4),
		std::make_tuple(11, 10, 2),
		std::make_tuple(10, 7, 6),
		std::make_tuple(7, 1, 8),
		std::make_tuple(3, 9, 4),
		std::make_tuple(3, 4, 2),
		std::make_tuple(3, 2, 6),
		std::make_tuple(3, 6, 8),
		std::make_tuple(3, 8, 9),
		std::make_tuple(4, 9, 5),
		std::make_tuple(2, 4, 11),
		std::make_tuple(6, 2, 10),
		std::make_tuple(8, 6, 7),
		std::make_tuple(9, 8, 1)
	};

	for (int i = 0; i < 5; ++i) {
		std::vector<std::tuple<size_t, size_t, size_t>> tmpfaces;
		tmpfaces.reserve(faces.size() * 4);
		for (auto& f : faces) {
			points.push_back(get_middle_point(points[std::get<0>(f)], points[std::get<1>(f)]));
			points.push_back(get_middle_point(points[std::get<1>(f)], points[std::get<2>(f)]));
			points.push_back(get_middle_point(points[std::get<2>(f)], points[std::get<0>(f)]));
			size_t idxa = points.size() - 3;
			size_t idxb = points.size() - 2;
			size_t idxc = points.size() - 1;
			tmpfaces.push_back(std::make_tuple(std::get<0>(f), idxa, idxc));
			tmpfaces.push_back(std::make_tuple(std::get<1>(f), idxb, idxa));
			tmpfaces.push_back(std::make_tuple(std::get<2>(f), idxc, idxb));
			tmpfaces.push_back(std::make_tuple(idxa, idxb, idxc));
		}
		faces = tmpfaces;
	}

	for(const std::string & path : {"uos", "uosr", "uos_rgb", "xyz", "xyzr", "xyz_rgb"}) {
		mkdir(path.c_str(), S_IRWXU|S_IRWXG|S_IRWXO);
		std::ofstream out_frames(path+"/scan000.frames");
		out_frames << "1 0 0 0 0 1 0 0 0 0 1 0 -0 0 0 1 2" << std::endl;
		out_frames.close();
		
		std::ofstream out_pose(path+"/scan000.pose");
		out_pose << "0 0 0" << std::endl;
		out_pose << "0 0 0" << std::endl;
		out_pose.close();
	}

	std::ofstream out_uos("uos/scan000.3d");
	std::ofstream out_uosr("uosr/scan000.3d");
	std::ofstream out_uos_rgb("uos_rgb/scan000.3d");
	std::ofstream out_xyz("xyz/scan000.xyz");
	std::ofstream out_xyzr("xyzr/scan000.xyz");
	std::ofstream out_xyz_rgb("xyz_rgb/scan000.xyz");
	for (auto& p: points) {
		out_uos << std::get<0>(p) << " " << std::get<1>(p) << " " << std::get<2>(p) << std::endl;
		out_uosr << std::get<0>(p) << " " << std::get<1>(p) << " " << std::get<2>(p)
			<< " " << (long)(((std::get<0>(p)+1)+(std::get<1>(p)+1)+(std::get<2>(p)+1))/6.0*255)
			<< std::endl;
		out_uos_rgb << std::get<0>(p) << " " << std::get<1>(p) << " " << std::get<2>(p)
			<< " " << (long)round((std::get<0>(p)+1)/2.0*255)
			<< " " << (long)round((std::get<1>(p)+1)/2.0*255)
			<< " " << (long)round((std::get<2>(p)+1)/2.0*255)
			<< std::endl;
		out_xyz << std::get<0>(p)/100.0 << " " << std::get<1>(p)/100.0 << " " << std::get<2>(p)/100.0 << std::endl;
		out_xyzr << std::get<0>(p)/100.0 << " " << std::get<1>(p)/100.0 << " " << std::get<2>(p)/100.0
			<< " " << (long)(((std::get<0>(p)+1)+(std::get<1>(p)+1)+(std::get<2>(p)+1))/6.0*255)
			<< std::endl;
		out_xyz_rgb << std::get<0>(p)/100.0 << " " << std::get<1>(p)/100.0 << " " << std::get<2>(p)/100.0
			<< " " << (long)round((std::get<0>(p)+1)/2.0*255)
			<< " " << (long)round((std::get<1>(p)+1)/2.0*255)
			<< " " << (long)round((std::get<2>(p)+1)/2.0*255)
			<< std::endl;
	}
	out_uos.close();
	out_uosr.close();
	out_uos_rgb.close();
	out_xyz.close();
	out_xyzr.close();
	out_xyz_rgb.close();

	return 0;
}
