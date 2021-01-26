#ifndef __DETECTCYLINDER_DETECTCYLINDER_H__
#define __DETECTCYLINDER_DETECTCYLINDER_H__
#include <boost/program_options.hpp>

#ifdef _WIN32
#define strcasecmp _stricmp
#define strncasecmp _strnicmp
#include <direct.h>
#define mkdir(path,mode) _mkdir (path)
#else
#include <strings.h>
#endif

#include <scanserver/clientInterface.h>

#include <slam6d/io_types.h>
#include <slam6d/globals.icc>
#include <slam6d/scan.h>
#include <slam6d/normals.h>

#include <detectCylinder/cylinderDetector.h>

void validate(boost::any& v, const std::vector<std::string>& values, IOType*, int);
void parse_options(int argc, char **argv, int &start, int &end, bool &scanserver, string &dir, IOType &iotype);


#endif
