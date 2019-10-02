/*
 * io_tpyes implementation
 *
 * Copyright (C) Thomas Escher, Kai Lingemann, Andreas Nuechter
 *
 * Released under the GPL version 3.
 *
 */


#include "slam6d/io_types.h"
#include "slam6d/globals.icc"

#ifdef _MSC_VER
//#include <string.h> // TODO: TEST
#define strcasecmp _stricmp
#define strncasecmp _strnicmp
#else
#include <strings.h>
#endif

#include <stdexcept>
#include <string>
IOType formatname_to_io_type(const char * string)
{
  if (strcasecmp(string, "ais") == 0) return AIS;
  else if (strcasecmp(string, "asc") == 0) return ASC;
  else if (strcasecmp(string, "faro_xyz_rgbr") == 0) return FARO_XYZ_RGBR;
  else if (strcasecmp(string, "front") == 0) return FRONT;
  else if (strcasecmp(string, "iais") == 0) return IAIS;
  else if (strcasecmp(string, "ifp") == 0) return IFP;
  else if (strcasecmp(string, "ks") == 0) return KS;
  else if (strcasecmp(string, "ks_rgb") == 0) return KS_RGB;
  else if (strcasecmp(string, "las") == 0) return LAZ;
  else if (strcasecmp(string, "laz") == 0) return LAZ;
  else if (strcasecmp(string, "leica") == 0) return LEICA;
  else if (strcasecmp(string, "leica_xyzr") == 0) return LEICA_XYZR;
  else if (strcasecmp(string, "oct") == 0) return OCT;
  else if (strcasecmp(string, "old") == 0) return OLD;
  else if (strcasecmp(string, "pci") == 0) return PCI;
  else if (strcasecmp(string, "pcl") == 0) return PCL;
  else if (strcasecmp(string, "ply") == 0) return PLY;
  else if (strcasecmp(string, "pts") == 0) return PTS;
  else if (strcasecmp(string, "ptsr") == 0) return PTSR;
  else if (strcasecmp(string, "pts_rgb") == 0) return PTS_RGB;
  else if (strcasecmp(string, "pts_rgbr") == 0) return PTS_RGBR;
  else if (strcasecmp(string, "pts_rrgb") == 0) return PTS_RRGB;
  else if (strcasecmp(string, "riegl_bin") == 0) return RIEGL_BIN;
  else if (strcasecmp(string, "riegl_project") == 0) return RIEGL_PROJECT;
  else if (strcasecmp(string, "riegl_rgb") == 0) return RIEGL_RGB;
  else if (strcasecmp(string, "riegl_txt") == 0) return RIEGL_TXT;
  else if (strcasecmp(string, "rts") == 0) return RTS;
  else if (strcasecmp(string, "rts_map") == 0) return RTS_MAP;
  else if (strcasecmp(string, "rxp") == 0) return RXP;
  else if (strcasecmp(string, "stl") == 0) return STL;
  else if (strcasecmp(string, "txyzr") == 0) return TXYZR;
  else if (strcasecmp(string, "uos") == 0) return UOS;
  else if (strcasecmp(string, "uosr") == 0) return UOSR;
  else if (strcasecmp(string, "cad") == 0) return UOS_CAD;
  else if (strcasecmp(string, "uos_frames") == 0) return UOS_FRAMES;
  else if (strcasecmp(string, "uos_map") == 0) return UOS_MAP;
  else if (strcasecmp(string, "uos_map_frames") == 0) return UOS_MAP_FRAMES;
  else if (strcasecmp(string, "uos_rgb") == 0) return UOS_RGB;
  else if (strcasecmp(string, "uos_rgbr") == 0) return UOS_RGBR;
  else if (strcasecmp(string, "uos_rrgb") == 0) return UOS_RRGB;
  else if (strcasecmp(string, "uos_rrgbt") == 0) return UOS_RRGBT;
  else if (strcasecmp(string, "velodyne") == 0) return VELODYNE;
  else if (strcasecmp(string, "velodyne_frames") == 0) return VELODYNE_FRAMES;
  else if (strcasecmp(string, "wrl") == 0) return WRL;
  else if (strcasecmp(string, "x3d") == 0) return X3D;
  else if (strcasecmp(string, "xyz") == 0) return XYZ;
  else if (strcasecmp(string, "xyzr") == 0) return XYZR;
  else if (strcasecmp(string, "xyz_rgb") == 0) return XYZ_RGB;
  else if (strcasecmp(string, "xyz_rgbr") == 0) return XYZ_RGBR;
  else if (strcasecmp(string, "xyz_rrgb") == 0) return XYZ_RRGB;
  else if (strcasecmp(string, "zahn") == 0) return ZAHN;
  else if (strcasecmp(string, "zuf") == 0) return ZUF;
  else if (strcasecmp(string, "uos_normal") == 0) return UOS_NORMAL;
  else throw std::runtime_error(std::string("Io type ") + string + std::string(" is unknown"));
}

const char * io_type_to_libname(IOType  type)
{
  switch (type) {
  case AIS:
    return "scan_io_ais";
  case ASC:
    return "scan_io_asc";
  case FARO_XYZ_RGBR:
    return "scan_io_faro_xyz_rgbr";
  case FRONT:
    return "scan_io_front";
  case IAIS:
    return "scan_io_iais";
  case IFP:
    return "scan_io_ifp";
  case KS:
    return "scan_io_ks";
  case KS_RGB:
    return "scan_io_ks_rgb";
  case LAZ:
    return "scan_io_laz";
  case LEICA:
    return "leica_txt";
  case LEICA_XYZR:
    return "scan_io_leica_xyzr";
  case OCT:
    return "scan_io_oct";
  case OLD:
    return "scan_io_old";
  case PCI:
    return "pci";
  case PCL:
    return "pcl";
  case PLY:
    return "scan_io_ply";
  case PTS:
    return "scan_io_pts";
  case PTSR:
    return "scan_io_ptsr";
  case PTS_RGB:
    return "scan_io_pts_rgb";
  case PTS_RGBR:
    return "scan_io_pts_rgbr";
  case PTS_RRGB:
    return "scan_io_pts_rrgb";
  case RIEGL_BIN:
    return "scan_io_riegl_bin";
  case RIEGL_PROJECT:
    return "scan_io_riegl_project";
  case RIEGL_RGB:
    return "scan_io_riegl_rgb";
  case RIEGL_TXT:
    return "scan_io_riegl_txt";
  case RTS:
    return "scan_io_rts";
  case RTS_MAP:
    return "scan_io_rts_map";
  case RXP:
    return "scan_io_rxp";
  case STL:
    return "stl";
  case TXYZR:
    return "scan_io_txyzr";
  case UOS:
    return "scan_io_uos";
  case UOSR:
    return "scan_io_uosr";
  case UOS_CAD:
    return "cad";
  case UOS_FRAMES:
    return "scan_io_uos_frames";
  case UOS_MAP:
    return "scan_io_uos_map";
  case UOS_MAP_FRAMES:
    return "scan_io_uos_map_frames";
  case UOS_RGB:
    return "scan_io_uos_rgb";
  case UOS_RGBR:
    return "scan_io_uos_rgbr";
  case UOS_RRGB:
    return "scan_io_uos_rrgb";
  case UOS_RRGBT:
    return "scan_io_uos_rrgbt";
  case VELODYNE:
    return "scan_io_velodyne";
  case VELODYNE_FRAMES:
    return "scan_io_velodyne_frames";
  case WRL:
    return "scan_io_wrl";
  case X3D:
    return "scan_io_x3d";
  case XYZ:
    return "scan_io_xyz";
  case XYZR:
    return "scan_io_xyzr";
  case XYZ_RGB:
    return "scan_io_xyz_rgb";
  case XYZ_RGBR:
    return "scan_io_xyz_rgbr";
  case XYZ_RRGB:
    return "scan_io_xyz_rrgb";
  case ZAHN:
    return "scan_io_zahn";
  case ZUF:
    return "scan_io_zuf";
  case UOS_NORMAL:
    return "scan_io_uos_normal";
  default:
    throw std::runtime_error(std::string("Io type ") + to_string(type) + std::string(" could not be matched to a library name"));
  }
}

//TODO check if all file formats are included
bool supportsReflectance(const IOType iotype) {
  switch (iotype) {
    case UOSR:
    case UOS_RRGBT:
    case UOS_RRGB:
    case RIEGL_TXT:
    case RIEGL_PROJECT:
    case RXP:
    case TXYZR:
    case XYZR:
    case XYZ_RRGB:
    case FARO_XYZ_RGBR:
    case LEICA_XYZR:
      return true;
      break;
    default:
      break;
  }
  return false;
}

//TODO check if all file formats are included
bool supportsColor(const IOType iotype) {
  switch (iotype) {
    case UOS_RGB:
    case UOS_RRGBT:
    case RIEGL_RGB:
    case XYZ_RGB:
    case KS_RGB:
    case PLY:
      return true;
      break;
    default:
       break;
  }
  return false;
}

bool supportsNormals(const IOType iotype) {
  switch(iotype) {
    case UOS_NORMAL:
      return true;
      break;
    default:
      break;
  }
  return false;
}

IODataType operator|=(IODataType a, IODataType b)
{
	return static_cast<IODataType>(static_cast<unsigned int>(a) | static_cast<unsigned int>(b));
}

IODataType operator|(IODataType a, IODataType b)
{
	return static_cast<IODataType>(static_cast<unsigned int>(a) | static_cast<unsigned int>(b));
}
