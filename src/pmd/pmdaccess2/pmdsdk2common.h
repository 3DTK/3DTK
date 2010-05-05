/*****************************************************************************
 * PMDSDK 2
 *
 * Copyright (c) 2006-2007 PMD Technologies GmbH
 * All Rights Reserved.
 *
 * File: pmdsdk2common.h
 * Author: Martin Profittlich
 * Created: 20060530
 *
 *****************************************************************************/

#ifndef MSDK2COMMON_H_82635748816555424683995524346951
#define MSDK2COMMON_H_82635748816555424683995524346951

/// \addtogroup core
/// @{

// Includes

#include <stddef.h>

// Macros

#ifdef __cplusplus
/// Mark beginning of extern "C" block
# define BEGIN_EXTERN_C extern "C" {
/// Mark ending of extern "C" block
# define END_EXTERN_C }
#else
/// Mark beginning of extern "C" block
# define BEGIN_EXTERN_C
/// Mark ending of extern "C" block
# define END_EXTERN_C
#endif

#ifdef __cplusplus
/// Mark beginning of pmd namespace
# define BEGIN_PMD_NAMESPACE namespace pmd {
/// Mark ending of pmd namespace
# define END_PMD_NAMESPACE }
#endif

#ifndef _WIN32
/// No export needed for non-windows platforms
# define PMD_EXPORT
#else
/// Win32 export definition (only if not already defined as dllexport)
# ifndef PMD_EXPORT
#  define PMD_EXPORT  __declspec(dllimport)
# endif
#endif

// Types

/// Single precision float
typedef float PMDFloat32;
/// Double precision float
typedef double PMDFloat64;

/// Standard float
#ifndef PMDFloat
typedef PMDFloat32 PMDFloat;
#endif

/// Unsigned 32-bit integer
typedef unsigned PMDUInt32;
/// Signed 32-bit integer
typedef signed int PMDSInt32;

/// Unsigned 16-bit integer
typedef unsigned short PMDUInt16;
/// Signed 16-bit integer
typedef signed short PMDSInt16;

/// Unsigned 8-bit integer
typedef unsigned char PMDUInt8;
/// Signed 8-bit integer
typedef signed char PMDSInt8;

/// Enumeration for retrieving valid integration times and modulation
/// frequencies
typedef enum { AtLeast, AtMost, CloseTo } Proximity;

// Resultcodes

/// \addtogroup StatusCodes
/// @{

/// Success
#define PMD_OK 0

/// Runtime error
#define PMD_RUNTIME_ERROR 1024
/// Generic error
#define PMD_GENERIC_ERROR 1025
/// Connection lost
#define PMD_DISCONNECTED 1026
/// An invalid value was given
#define PMD_INVALID_VALUE 1027

/// Program error
#define PMD_LOGIC_ERROR 2048
/// Handle not known (internal error)
#define PMD_UNKNOWN_HANDLE 2049
/// Requested functionality not implemented
#define PMD_NOT_IMPLEMENTED 2050
/// Index or value out of range
#define PMD_OUT_OF_BOUNDS 2051

/// Could not get resource
#define PMD_RESOURCE_ERROR 4096
/// Could not open file
#define PMD_FILE_NOT_FOUND 4097
/// Could not connect to or open the data source
#define PMD_COULD_NOT_OPEN 4098
/// Could not retrieve the requested data
#define PMD_DATA_NOT_FOUND 4099
/// There is no more data left
#define PMD_END_OF_DATA 4100

// DEPRECATED
#ifndef PMD_NO_DEPRECATED
#define PMD_COULD_NOT_CONNECT 4098
#endif

/// @}

// Limits

// Versions

/// Version 1.0.0 of the plugin interface
#define PMD_INTERFACE_VERSION_1_0_0 0x00010000
/// Version 1.1.0 of the plugin interface
#define PMD_INTERFACE_VERSION_1_1_0 0x00010100
/// Version 1.1.1 of the plugin interface
#define PMD_INTERFACE_VERSION_1_1_1 0x00010101
/// Version 1.2.0 of the plugin interface
#define PMD_INTERFACE_VERSION_1_2_0 0x00010200
/// Version 1.2.1 of the plugin interface
#define PMD_INTERFACE_VERSION_1_2_1 0x00010201
/// Version 1.3.0 of the plugin interface
#define PMD_INTERFACE_VERSION_1_3_0 0x00010300
/// The current version of the plugin interface
#define PMD_CURRENT_INTERFACE_VERSION PMD_INTERFACE_VERSION_1_3_0

// Helper functions

BEGIN_EXTERN_C

/// Generate a new unique ID
unsigned pmdNewGUID ();

struct PMDDataDescription;
/// Convert byte order of a PMDDataDescription structure.
void pmdHelperConvertDDByteOrder (struct PMDDataDescription * dd);

END_EXTERN_C

/// @}

#endif
