/*
 * scan_io_xyz implementation
 *
 * Copyright (C) Andreas Nuechter, Dorit Borrmann
 *
 * Released under the GPL version 3.
 *
 */


/**
 * @file scan_io_xyz.cc
 * @brief IO of a 3D scan in xyz file format (right-handed coordinate system,
 * with z as height)
 * @author Andreas Nuechter. Jacobs University Bremen, Germany.
 * @author Dorit Borrmann. Jacobs University Bremen, Germany.
 */

#include "scanio/scan_io_xyz.h"

const char* ScanIO_xyz::data_suffix = ".xyz";
ScanDataTransform_xyz scanio_xyz_tf;
ScanDataTransform& ScanIO_xyz::transform2uos = scanio_xyz_tf;


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
  return new ScanIO_xyz;
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
