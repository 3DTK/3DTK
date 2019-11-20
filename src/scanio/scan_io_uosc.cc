/*
 * scan_io_uosc implementation
 *
 * Copyright (C) Michael Neumann, Billy Okal
 *
 * Released under the GPL version 3.
 *
 */


/**
 * @file scan_io_uosc.cc
 * @brief IO of a 3D scan in uos file format plus a class (type)
 * @author Michael Neumann. Universiy of Wuerzburg, Germany.
 * @author Billy Okal. Jacobs University Bremen, Germany.
 */

#include "scanio/scan_io_uosc.h"

IODataType ScanIO_uosc::spec[] = { DATA_XYZ, DATA_XYZ, DATA_XYZ, DATA_TYPE, DATA_TERMINATOR };


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
  return new ScanIO_uosc;
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
