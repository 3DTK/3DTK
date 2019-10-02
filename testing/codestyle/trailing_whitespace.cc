#include <boost/filesystem.hpp>
#include <boost/regex.hpp>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>

std::vector<std::string> whitelist = {
#ifdef _WIN32
	".*\\\\include\\\\riegl\\\\.*\\.a",
	".*\\\\include\\\\riegl\\\\.*\\.hpp",
	".*\\\\include\\\\riegl\\\\.*\\.so",
	".*\\\\include\\\\show\\\\url\\.png",
	".*\\\\src\\\\cuda\\\\grid_kernel.cu",
	".*\\\\src\\\\grid\\\\docu.tex",
	".*\\\\src\\\\mesh\\\\README.md",
	".*\\\\src\\\\pmd\\\\README",
	".*\\\\src\\\\pmd\\\\o3d.L32\\.pcp",
	".*\\\\src\\\\pmd\\\\offline\\\\pmdc.conf",
	".*\\\\src\\\\pmd\\\\offline\\\\rename",
	".*\\\\src\\\\pmd\\\\pmdc.conf",
	".*\\\\src\\\\pmd\\\\pose\\\\dat\\\\commas2dots",
	".*\\\\src\\\\pmd\\\\pose\\\\djvm\\.ttf",
	".*\\\\src\\\\pmd\\\\pose\\\\pmdc.conf",
	".*\\\\src\\\\slam6d\\\\testICPortho.m",
	".*\\\\src\\\\spherical_quadtree\\\\.*\\.pyc",
	".*\\\\src\\\\veloslam\\\\SegIter.model",
#else
	".*/include/riegl/.*\\.a",
	".*/include/riegl/.*\\.hpp",
	".*/include/riegl/.*\\.so",
	".*/include/show/url\\.png",
	".*/src/cuda/grid_kernel.cu",
	".*/src/grid/docu.tex",
	".*/src/mesh/README.md",
	".*/src/pmd/README",
	".*/src/pmd/o3d.L32\\.pcp",
	".*/src/pmd/offline/pmdc.conf",
	".*/src/pmd/offline/rename",
	".*/src/pmd/pmdc.conf",
	".*/src/pmd/pose/dat/commas2dots",
	".*/src/pmd/pose/djvm\\.ttf",
	".*/src/pmd/pose/pmdc.conf",
	".*/src/slam6d/testICPortho.m",
	".*/src/spherical_quadtree/.*\\.pyc",
	".*/src/veloslam/SegIter.model",
#endif
};

int main(int argc, char* argv[])
{
	std::vector<std::string> failed_files;

	boost::regex trailing_whitespace(".*[ \t]+$");

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
				boost::regex re(p);
				if (boost::regex_match(boost::filesystem::canonical(path).string(), re)) {
					is_whitelisted = true;
					break;
				}
			}
			if (is_whitelisted) {
				continue;
			}

			std::ifstream ifs(path.string());
			std::string line;

			int i = 1;
			bool found_trailing = false;
			while (std::getline(ifs, line)) {
				if (boost::regex_match(line, trailing_whitespace)) {
					std::cerr << path.string() << ":" << i << " trailing whitespace" << std::endl;
					found_trailing = true;
				}
				++i;
			}
			if (found_trailing) {
				failed_files.push_back(path.string());
			}
		}
	}

	if (failed_files.size() > 0) {
		std::cerr << "fix them with: sed --in-place 's/[[:space:]]\\+$//'";
		for (const auto &f : failed_files) {
			std::cerr << " " << f;
		}
		std::cerr << std::endl;
		return 1;
	} else {
		return 0;
	}
}
