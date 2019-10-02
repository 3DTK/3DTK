/*
 * scan_io_ptsr implementation
 *
 * Copyright (C) Andreas Nuechter, Dorit Borrmann
 *
 * Released under the GPL version 3.
 *
 */


/**
 * @file scan_io_ptsr.cc
 * @brief IO of a 3D scan in pts file format (right-handed coordinate system,
 * with x from left to right, y from bottom to up, z from front to back)
 * @author Hamidreza Houshiar. DenkmalDaten Winkler KG. Co. Muenster, Germany
 */

#include "scanio/scan_io_ptsr.h"
const char* ScanIO_ptsr::data_suffix = ".pts";
IODataType ScanIO_ptsr::spec[] = { DATA_XYZ, DATA_XYZ, DATA_XYZ,
    DATA_REFLECTANCE, DATA_TERMINATOR };
ScanDataTransform_pts scanio_ptsr_tf;
ScanDataTransform& ScanIO_ptsr::transform2uos = scanio_ptsr_tf;


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
  return new ScanIO_ptsr;
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
