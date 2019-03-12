/**
 * @file
 * @brief Scan types and mapping functions.
 *
 * @author Thomas Escher, Billy Okal, Dorit Borrmann
 */

#ifndef IO_TYPES_H
#define IO_TYPES_H

//! IO types for file formats, distinguishing the use of ScanIOs
enum IOType {
  AIS, ASC, FARO_XYZ_RGBR, FRONT, IAIS, IFP, KS, KS_RGB, LAZ, LEICA, LEICA_XYZR, OCT, OLD, PCI, PCL, PLY, PTS, PTSR, PTS_RGB, PTS_RGBR, PTS_RRGB, RIEGL_BIN, RIEGL_PROJECT, RIEGL_RGB, RIEGL_TXT, RTS, RTS_MAP, RXP, STL, TXYZR, UOS, UOSR, UOS_CAD, UOS_FRAMES, UOS_MAP, UOS_MAP_FRAMES, UOS_RGB, UOS_RGBR, UOS_RRGB, UOS_RRGBT, VELODYNE, VELODYNE_FRAMES, WRL, X3D, XYZ, XYZR, XYZ_RGB, XYZ_RGBR, XYZ_RRGB, ZAHN, ZUF, UOS_NORMAL};

//! Data channels in the scans
enum IODataType : unsigned int {
  DATA_TERMINATOR = 0,
  DATA_DUMMY = 1<<0,
  DATA_XYZ = 1<<1,
  DATA_RGB = 1<<2,
  DATA_REFLECTANCE = 1<<3,
  DATA_TEMPERATURE = 1<<4,
  DATA_AMPLITUDE = 1<<5,
  DATA_TYPE = 1<<6,
  DATA_DEVIATION = 1<<7,
  DATA_NORMAL = 1<<8
};

IODataType operator|=(IODataType a, IODataType b);

IODataType operator|(IODataType a, IODataType b);

IOType formatname_to_io_type(const char * string);

const char * io_type_to_libname(IOType type);

bool supportsColor(const IOType iotype);

bool supportsReflectance(const IOType iotype);

bool supportsNormals(const IOType iotype);

#endif //IO_TYPES_H
