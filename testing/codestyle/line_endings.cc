#include <boost/filesystem.hpp>
#include <boost/regex.hpp>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>

std::vector<std::string> whitelist = {
#ifdef _WIN32
	".*\\\\include\\\\riegl\\\\.*\\.a",
	".*\\\\include\\\\riegl\\\\.*\\.so",
	".*\\\\include\\\\show\\\\url\\.png",
	".*\\\\src\\\\pmd\\\\o3d.L32\\.pcp",
	".*\\\\src\\\\pmd\\\\pose\\\\djvm\\.ttf",
	".*\\\\src\\\\spherical_quadtree\\\\.*\\.pyc",
#else
	".*/include/riegl/.*\\.a",
	".*/include/riegl/.*\\.so",
	".*/include/show/url\\.png",
	".*/src/pmd/o3d.L32\\.pcp",
	".*/src/pmd/pose/djvm\\.ttf",
	".*/src/spherical_quadtree/.*\\.pyc",
#endif
};

int main(int argc, char* argv[])
{
	bool has_error = false;

	for (int i = 1; i < argc; ++i) {
		// c++17 added boost::filesystem as std::filesystem
		for (const auto& dirEntry : boost::filesystem::recursive_directory_iterator(argv[i])) {
			boost::filesystem::path path(dirEntry.path());
			if (boost::filesystem::is_directory(path)) {
				continue;
			}
			if (!boost::filesystem::is_regular_file(path)) {
				std::cerr << "is not a directory or regular file: " << dirEntry.path() << std::endl;
				return 1;
			}
			bool is_whitelisted = false;
			for (const auto &p : whitelist) {
				boost::regex re(p, boost::regex_constants::basic);
				if (boost::regex_match(boost::filesystem::canonical(path).string(), re)) {
					is_whitelisted = true;
					break;
				}
			}
			if (is_whitelisted) {
				continue;
			}
			std::ifstream ifs(path.string(), std::ios::binary);
			std::string str((std::istreambuf_iterator<char>(ifs)), std::istreambuf_iterator<char>());
			ifs.close();

			if (str.find("\r") != std::string::npos) {
				std::cerr << "found carriage return character in " << path << std::endl;
				has_error = true;
			}
		}
	}

	if (has_error) {
		return 1;
	} else {
		return 0;
	}
}
