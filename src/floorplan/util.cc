/**
 * @file util.cc
 *
 * @auhtor Remus Claudiu Dumitru <r.dumitru@jacobs-university.de>
 * @date 13 Feb 2012
 *
 */

//==============================================================================
//  Includes.
//==============================================================================
#include "floorplan/util.h"

// C includes.
#include <sys/stat.h>       // stat()
#ifdef _WIN32
#include "dirent.h"
#include <direct.h>
#define mkdir(path,mode) _mkdir (path)
#else
#include <unistd.h>
#endif
#include <math.h>

#include <stdexcept>
using namespace std;

//==============================================================================
//  Implementation.
//==============================================================================
bool floorplan::fileExists(const string& fileName) {
    struct stat buffer ;
    if (stat(fileName.c_str(), &buffer)) return false;   // ret 0 for found
    return true;
}

bool floorplan::fileIsDir(const string& fileName) {
    struct stat buffer;
    int status = stat(fileName.c_str(), &buffer);

    if (status != 0 || S_ISREG(buffer.st_mode)) {
        return false;
    }

    if (S_ISDIR(buffer.st_mode)) {
        return true;
    }

    return false;
}

bool floorplan::makeDir(const string& path) {
	if (fileExists(path) && fileIsDir(path)) {
		return true;
	}

	return (mkdir(path.c_str(), S_IRWXU | S_IRWXG | S_IRWXO) == 0 ? true : false);
}
