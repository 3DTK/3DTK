/**
 * @file
 * @brief Server/Client wide definitions.
 *
 * If running multiple instances of the scanserver, one can change these shared memory names individually.
 *
 * @author Thomas Escher
 */

#ifndef SCANSERVER_DEFINES_H
#define SCANSERVER_DEFINES_H

//! shared memory for SharedScans, Frames and communications
#define SHM_NAME_DATA "3dtk_scanserver_data"

//! CacheManager/CacheObject only shared memory
#define SHM_NAME_CACHE "3dtk_scanserver_cache"

#endif //SCANSERVER_DEFINES_H
