#include <zip.h>
#include <iostream>
#include <string>

int main(int argc, char* argv[])
{
	if (argc != 2) {
		std::cerr << "usage: " << argv[0] << " basepath" << std::endl;
		std::cerr << std::endl;
		std::cerr << "The basepath must include scan000.pose, scan000.3d and scan000.frames" << std::endl;
		std::cerr << std::endl;
		std::cerr << "The program will put a normals.zip into the basepath containing these files." << std::endl;
		return 1;
	}

	std::string zip_fname = std::string(argv[1]) + "/normals.zip";

	int error;
	int flags = ZIP_CREATE;
	struct zip *archive = zip_open(zip_fname.c_str(), flags, &error);
	if (archive == nullptr) {
		std::cerr << "zip_open() failed" << std::endl;
		return 1;
	}

	for (auto& ext : {"pose", "3d"}) {
		std::string fname = std::string(argv[1]) + "/normals/scan000." + ext;

		zip_source *source = zip_source_file(archive, fname.c_str(), 0, 0);
		if (source == nullptr) {
			std::cerr << "zip_source_file() failed for " << fname << std::endl;
			return 1;
		}

		std::string zipname = std::string("normals/scan000.") + ext;
		zip_int64_t index = zip_file_add(archive, zipname.c_str(), source, 0);
		if (index < 0) {
			std::cerr << "zip_file_add() failed for " << fname << std::endl;
			return 1;
		}
	}

	if (zip_close(archive) < 0) {
		std::cerr << "zip_close() failed: " << zip_strerror(archive) << std::endl;
		zip_discard(archive);
		return 1;
	}

	return 0;
}
