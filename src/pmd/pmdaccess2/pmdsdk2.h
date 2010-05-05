/* 
 * PMDSDK 2
 *
 * File: pmdsdk2.h
 * Author: Martin Profittlich
 * Version: 1.0 
 *
 * General header file for applications using the PMDSDK 2.0.
 * Contains all necessary definitions and prototypes.
 *
 * Copyright (c) 2006-2007 PMD Technologies GmbH.
 * All Rights Reserved.
 *
 */

//
// Differences from MiniSDK 1.0:
//
// - Distance, amplitude and intensity data type is float (was double)
// - pmdGetRaw() is now pmdGetSourceData()
// - Error code values changed
// - User is responsible for memory (de-)allocation now
// - Info commands removed, introduced pmdSourceCommand instead
// - Plugins!
// - Multiple integration times and modulation frequencies
// - Only one connection command, camera identification via plugin/parameter
//

#ifndef PMDMSDK2_H_INCLUDED_2503171013
#define PMDMSDK2_H_INCLUDED_2503171013

#include "pmddatadescription.h"
#include "pmdheader.h"

#ifdef _WIN32
# ifndef DLLSPEC
#  define DLLSPEC __declspec(dllimport)
# endif
#else
# ifdef DLLSPEC
#  undef DLLSPEC
# endif
# define DLLSPEC
#endif

#ifdef __cplusplus
extern "C" {
#endif

// TYPES

/// Handle for a camera connection.
///This handle is used for all subsequent operations.
///
//typedef void* PMDHandle;
typedef unsigned PMDHandle;

// FUNCTIONS

ImageHeaderInformation *retriveHeader();

int pmdGetDistancesAsync(PMDHandle hnd, float *data, size_t maxLen);
int pmdGetAmplitudesAsync(PMDHandle hnd, float * data, size_t maxLen);
int pmdGetIntensitiesAsync(PMDHandle hnd, float * data, size_t maxLen);


/// Connect to a PMD camera or other data source
///\param hnd Empty PMDHandle structure. On success, this value
///will contain the handle for subsequent operations.
///\param rplugin Path of the camera plugin
///\param rparam Parameter for the camera plugin
///\param pplugin Path of the processing plugin. If this is NULL, no porcessing plugin will be loaded.
///\param pparam Parameter for the processing plugin
///\return PMD_OK on success, errorcode otherwise
///
DLLSPEC int pmdOpen (PMDHandle * hnd, const char * rplugin, const char * rparam, const char * pplugin, const char * pparam);

/// Connect to a PMD camera or other data source without processing
///\param hnd Empty PMDHandle structure. On success, this value
///will contain the handle for subsequent operations.
///\param rplugin Path of the camera plugin
///\param rparam Parameter for the camera plugin
///\return PMD_OK on success, errorcode otherwise
///
DLLSPEC int pmdOpenSourcePlugin (PMDHandle * hnd, const char * rplugin, const char * rparam);

/// Disconnect and close the handle.
/// \param hnd Handle of the connection.
/// \return PMD_OK on success, errorcode otherwise
///
DLLSPEC int pmdClose (PMDHandle hnd);

/// Get an error description from the last error.
/// Error messages are stored per handle. A new error associated with PMDHandle A
/// does not overwrite the last error message associated with PMDHandle B.
/// \param hnd Handle of the connection/plugin.
/// \param error Memory to hold the error message.
/// \param maxLen Maximum length of the error message, including the terminating zero byte.
/// Longer messages will be truncated.
DLLSPEC int pmdGetLastError (PMDHandle hnd, char * error, size_t maxLen);

/// Retrieve the a new frame from the camera.
/// To obtain the actual data, use pmdGetSourceData, pmdGetDistances,
/// pmdGetAmplitudes etc.
/// \param hnd Handle of the connection.
/// \return PMD_OK on success, errorcode otherwise
///
DLLSPEC int pmdUpdate (PMDHandle hnd);

/// Set the integration time of the camera
/// \param hnd Handle of the connection.
/// \param idx Index of the integration time.
/// \param t Integration time in microseconds.
/// \return PMD_OK on success, errorcode otherwise
///
DLLSPEC int pmdSetIntegrationTime (PMDHandle hnd, unsigned idx, unsigned t);

/// Get the integration time of the camera
/// \param hnd Handle of the connection.
/// \param idx Index of the integration time.
/// \param t Pointer to a variable to contain the 
/// integration time in microseconds.
/// \return PMD_OK on success, errorcode otherwise
///
DLLSPEC int pmdGetIntegrationTime (PMDHandle hnd, unsigned * t, unsigned idx);

/// Get a supported integration time of the camera
/// \param hnd Handle of the connection.
/// \param idx Index of the integration time.
/// \param t The desired integration time
/// \param w Where to look for a valid integration time in respect to the desired
/// integration time (CloseTo, AtLeast or AtMost)
/// \param responsible Pointer to a variable to contain the 
/// integration time in microseconds.
/// \return PMD_OK on success, errorcode otherwise
///
DLLSPEC int pmdGetValidIntegrationTime (PMDHandle hnd, unsigned * result, unsigned idx, Proximity w, unsigned t);

/// Set the modulation frequency of the camera
/// \param hnd Handle of the connection.
/// \param idx Index of the modulation frequency.
/// \param f Modulation frequency in Hz.
/// \return PMD_OK on success, errorcode otherwise
///
DLLSPEC int pmdSetModulationFrequency (PMDHandle hnd, unsigned idx, unsigned f);

/// Get the modulation frequency of the camera
/// \param hnd Handle of the connection.
/// \param idx Index of the modulation frequency.
/// \param t Pointer to a variable to contain the 
/// modulation frequency in Hz.
/// \return PMD_OK on success, errorcode otherwise
///
DLLSPEC int pmdGetModulationFrequency (PMDHandle hnd, unsigned * f, unsigned idx);

/// Get a supported modulation frequency of the camera
/// \param hnd Handle of the connection.
/// \param idx Index of the modulation frequency.
/// \param f The desired modulation frequency
/// \param w Where to look for a valid modulation frequency in respect to the desired
/// modulation frequency (CloseTo, AtLeast or AtMost)
/// \param result Pointer to a variable to contain the 
/// modulation frequency in Hz.
/// \return PMD_OK on success, errorcode otherwise
///
DLLSPEC int pmdGetValidModulationFrequency (PMDHandle hnd, unsigned * result, unsigned idx, Proximity w, unsigned f);

/// Get the raw data from the current frame.
/// \param hnd Handle of the connection.
/// \param data Pointer to the memory to contain the address of the data.
/// \param maxLen Maximum length in bytes for the data
/// \return PMD_OK on success, errorcode otherwise
///
DLLSPEC int pmdGetSourceData (PMDHandle hnd, void * data, size_t maxLen);

/// Get the size in bytes of the current raw data frame.
/// \param hnd Handle of the connection.
/// \param size Will contain the size after the call
/// \return PMD_OK on success, errorcode otherwise
///
DLLSPEC int pmdGetSourceDataSize (PMDHandle hnd, size_t * size);

/// Get the description of the current raw data frame.
/// \param hnd Handle of the connection.
/// \param dd Will contain the PMDDataDescription after the call.
/// \return PMD_OK on success, errorcode otherwise
///
DLLSPEC int pmdGetSourceDataDescription (PMDHandle hnd, struct PMDDataDescription * dd);

/// Get the distance data from the current frame.
/// \param hnd Handle of the connection.
/// \param data Pointer to a block of memory to contain the data.
/// \return PMD_OK on success, errorcode otherwise
///
DLLSPEC int pmdGetDistances (PMDHandle hnd, float * data, size_t maxLen);

/// Get the amplitude data from the current frame.
/// \param hnd Handle of the connection.
/// \param data Pointer to a block of memory to contain the data.
/// \return PMD_OK on success, errorcode otherwise
///
DLLSPEC int pmdGetAmplitudes (PMDHandle hnd, float * data, size_t maxLen);

/// Get the intensity data from the current frame.
/// \param hnd Handle of the connection.
/// \param data Pointer to a block of memory to contain the data.
/// \return PMD_OK on success, errorcode otherwise
///
DLLSPEC int pmdGetIntensities (PMDHandle hnd, float * data, size_t maxLen);

/// Execute an source plugin-specific command.
/// \param hnd Handle of the connection.
/// \param result Pointer to a block of memory to contain the result string.
/// \param maxLen Maximum length of the result string, including terminating 0.
/// \param cmd The command to be executed.
///
DLLSPEC int pmdSourceCommand (PMDHandle hnd, char * result, size_t maxLen, const char * cmd);

/// Execute a processing plugin-specific command.
/// \param hnd Handle of the connection.
/// \param result Pointer to a block of memory to contain the result string.
/// \param maxLen Maximum length of the result string, including terminating 0.
/// \param cmd The command to be executed.
///
DLLSPEC int pmdProcessingCommand (PMDHandle hnd, char * result, size_t maxLen, const char * cmd);

// ADDITIONAL PROCESSING

DLLSPEC int pmdOpenProcessingPlugin (PMDHandle * hnd, const char * pplugin, const char * pparam);

DLLSPEC int pmdProcess (PMDHandle hnd, 
        unsigned numOut, struct PMDDataDescription * ddOut, void ** output,
        unsigned numIn, struct PMDDataDescription * ddIn, void ** input);

DLLSPEC int pmdCanProcess (PMDHandle hnd, unsigned * memNeeded, unsigned type, unsigned numFmt, struct PMDDataDescription * fmt);

// SPECIAL FUNCTIONS
// additional header file? inline code?
DLLSPEC int pmdConnectFireWire (PMDHandle * hnd, unsigned index);
DLLSPEC int pmdConnectASample (PMDHandle * hnd, const char * address);

#ifndef PMD_NO_DEPRECATED
// DEPRECATED
DLLSPEC int pmdConnect (PMDHandle * hnd, const char * rplugin, const char * rparam, const char * pplugin, const char * pparam);
DLLSPEC int pmdConnectOnlyRaw (PMDHandle * hnd, const char * rplugin, const char * rparam);
DLLSPEC int pmdDisconnect (PMDHandle hnd);
DLLSPEC int pmdOpenAccessPlugin (PMDHandle * hnd, const char * rplugin, const char * rparam);
DLLSPEC int pmdGetRawData (PMDHandle hnd, void * data, size_t maxLen);
DLLSPEC int pmdGetRawDataSize (PMDHandle hnd, size_t * size);
DLLSPEC int pmdGetRawDataDescription (PMDHandle hnd, struct PMDDataDescription * dd);
DLLSPEC int pmdPlatformCommand (PMDHandle hnd, char * result, size_t maxLen, const char * cmd);
DLLSPEC int pmdConfigureProcess (PMDHandle hnd, char * result, size_t maxLen, const char * cmd);
DLLSPEC int pmdGetInfo (PMDHandle hnd, char * result, size_t maxLen, const char * key);
#endif

#ifdef __cplusplus
}
#endif

#endif
