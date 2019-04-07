/*
 * scan_io_uos implementation
 *
 * Copyright (C) Thomas Escher, Kai Lingemann, Andreas Nuechter
 *
 * Released under the GPL version 3.
 *
 */


/**
 * @file
 * @brief Implementation of reading 3D scans
 * @author Kai Lingemann. Institute of Computer Science, University of Osnabrueck, Germany.
 * @author Andreas Nuechter. Institute of Computer Science, University of Osnabrueck, Germany.
 * @author Thomas Escher. Institute of Computer Science, University of Osnabrueck, Germany.
 */

#include "scanio/scan_io_uos.h"

/*
const char* ScanIO_uos::data_prefix = "scan";
const char* ScanIO_uos::data_suffix = ".3d";
const char* ScanIO_uos::pose_prefix = "scan";
const char* ScanIO_uos::pose_suffix = ".pose";

IODataType ScanIO_uos::spec[] = { DATA_XYZ, DATA_XYZ, DATA_XYZ, DATA_TERMINATOR };
ScanDataTransform_identity tf;
ScanDataTransform& ScanIO_uos::transform2uos[] = tf;
*/


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
  return new ScanIO_uos;
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
