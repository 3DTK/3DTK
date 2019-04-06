/*
 * scan_io_uosr implementation
 *
 * Copyright (C) Billy Okal
 *
 * Released under the GPL version 3.
 *
 */


/**
 * @file scan_io_uosr.cc
 * @brief IO of a 3D scan in uos file format plus a
 *        reflectance/intensity/temperature value
 * @author Billy Okal. Jacobs University Bremen, Germany.
 */

#include "scanio/scan_io_uos_normal.h"

IODataType ScanIO_uos_normal::spec[] = { DATA_XYZ, DATA_XYZ, DATA_XYZ, DATA_NORMAL, DATA_NORMAL, DATA_NORMAL, DATA_TERMINATOR };


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
  return new ScanIO_uos_normal;
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
