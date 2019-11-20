/*
 * scan_io_xyzc implementation
 *
 * Copyright (C) Michael Neumann, Andreas Nuechter
 *
 * Released under the GPL version 3.
 *
 */


/**
 * @file scan_io_xyzc.cc
 * @brief IO of a 3D scan in xyz file format plus a class (type)
 * @author Michael Neumann. University of Wuerzburg, Germany.
 * @author Andreas Nuechter. Jacobs University Bremen, Germany.
 */

#include "scanio/scan_io_xyzc.h"

const char* ScanIO_xyzc::data_suffix = ".xyz";
IODataType ScanIO_xyzc::spec[] = { DATA_XYZ, DATA_XYZ, DATA_XYZ,
        DATA_TYPE, DATA_TERMINATOR };
ScanDataTransform_xyz scanio_xyzc_tf;
ScanDataTransform& ScanIO_xyzc::transform2uos = scanio_xyzc_tf;

/**
 * class factory for object construction
 *
 * @return Pointer to new object
 */
#ifdef _MSC_VER
extern "C" __declspec(dllexport) ScanIO* create()
#else
extern "C" ScanIO* create()
#endif
{
  return new ScanIO_xyzc;
}

/**
 * class factory for object construction
 *
 * @return Pointer to new object
 */
#ifdef _MSC_VER
extern "C" __declspec(dllexport) void destroy(ScanIO *sio)
#else
extern "C" void destroy(ScanIO *sio)
#endif
{
  delete sio;
}

#ifdef _MSC_VER
BOOL APIENTRY DllMain(HANDLE hModule, DWORD dwReason, LPVOID lpReserved)
{
    return TRUE;
}
#endif

/* vim: set ts=4 sw=4 et: */
