/*****************************************************************************
 * PMDSDK 2
 * 
 * Copyright (c) 2006-2007 PMD Technologies GmbH
 * All Rights Reserved.
 *
 * File: pmddatadescription.h
 * Author: Martin Profittlich
 * Created: 20060808
 *
 *****************************************************************************/

#ifndef DATADESCRIPTION_H_672861066752896024487934289650
#define DATADESCRIPTION_H_672861066752896024487934289650

/// \addtogroup core
/// @{

#include "pmdsdk2common.h"

BEGIN_EXTERN_C


/// \addtogroup DataCodes
/// @{

// TODO: move defines to separate file

// raw data ID system:
// 0x00 0x00 0bORWWWWWW 0bRRRRDABD
// R = Reserved, must be 0
// O = Byte order (1 = Big Endian)
// W = Bit width
// D = Difference
// A = Channel A
// B = Channel B

/// A/B PMD data (e.g. PMD[vision] 3k-S)
#define PMD_RAW_L16_AB        0x00001006u
/// A/B PMD data (BigEndian) (e.g. CamCube)
#define PMD_RAW_B16_AB        0x00009006u
/// Difference and A/B PMD data
#define PMD_RAW_L16_DAB       0x0000100Eu
/// Difference and A/B PMD data (Big Endian) (e.g. USB-L)
#define PMD_RAW_B16_DAB    0x0000900Eu
/// A/B and difference PMD data
#define PMD_RAW_L16_ABD       0x00001007u
/// A/B and difference PMD data (Big Endian) (e.g. PMD[vision] A2 w/o addition information)
#define PMD_RAW_B16_ABD    0x00009007u
/// Packed PMD difference data (e.g. PMD[vision] 19k)
#define PMD_RAW_P12_D         0x00000C01u
/// PMD difference data (e.g. PMD[vision] 64)
#define PMD_RAW_L16_D         0x00001001u

/// PMD difference data with saturation information
//#define PMD_RAW_L16D_SAT     1005u
/// A/B, difference and information PMD data (e.g. PMD[vision] A2 w/ additional information)
//#define PMD_RAW_L16ABD_AUX   1006u

/// Unknown data
#define PMD_UNKNOWN_DATA   0x00000000u

/// 32-bit floating point distances in meters
#define PMD_DISTANCE_LF32       0x00010001u
/// 32-bit floating point amplitudes
#define PMD_AMPLITUDE_LF32      0x00010002u
/// 32-bit floating point intensities
#define PMD_INTENSITY_LF32      0x00010004u
/// 32-bit floating point reflectivities
#define PMD_REFLECTIVITY_LF32   0x00010008u
/// 16-bit boolean information
#define PMD_FLAGS_16            0x00010010u

/// Cartesian X coordinates
#define PMD_X_COORD_LF32        0x00010020u
/// Cartesian Y coordinates
#define PMD_Y_COORD_LF32        0x00010040u
/// Cartesian Z coordinates
#define PMD_Z_COORD_LF32        0x00010080u

/// 16 bit distances in millimeters and amplitudes
#define PMD_DISTANCE_I16_AMPLITUDE_I16 0x00020003u
/// 16 bit distances in 100 micrometers and amplitudes
#define PMD_DISTANCE_100UM_I16_AMPLITUDE_I16 0x00020103u

/// A3 camera data v2.31
#define PMD_A3_DATA_2_31      0x00040003u
/// A3 camera data v4
#define PMD_A3_DATA_4         0x00040005u
/// A3 camera raw data
#define PMD_A3_RAWDATA_2_31   0x00040007u
/// A3 sub type
#define PMD_A3_DISTANCE       0x00000001u
/// A3 sub type
#define PMD_A3_AMPLITUDE      0x00000002u
/// A3 sub type
#define PMD_A3_CONFIDENCE     0x00000004u
/// A3 sub type
#define PMD_A3_COORDINATES    0x00000008u
/// A3 sub type
#define PMD_A3_OBJECT_LIST    0x00000010u

/// A3 camera data
#define PMD_O3D_DATA_1_7      0x00050003u
/// O3D sub type
#define PMD_O3D_DISTANCE       0x00000002u
/// O3D sub type
#define PMD_O3D_INTENSITY     0x00000004u
/// O3D sub type
#define PMD_O3D_STDDEV    0x00000010u
/// O3D sub type
#define PMD_O3D_VIEW_X    0x00000020u
/// O3D sub type
#define PMD_O3D_VIEW_Y    0x00000040u
/// O3D sub type
#define PMD_O3D_VIEW_Z    0x00000080u
/// O3D sub type
#define PMD_O3D_X    0x00000100u
/// O3D sub type
#define PMD_O3D_Y    0x00000200u
/// O3D sub type
#define PMD_O3D_Z    0x00000400u

/// user defined data type
#define PMD_USER_DEFINED_0   0xFFFF0000u
/// user defined data type
#define PMD_USER_DEFINED_1   0xFFFF0001u
/// user defined data type
#define PMD_USER_DEFINED_2   0xFFFF0002u
/// user defined data type
#define PMD_USER_DEFINED_3   0xFFFF0003u
/// user defined data type
#define PMD_USER_DEFINED_4   0xFFFF0004u
/// user defined data type
#define PMD_USER_DEFINED_5   0xFFFF0005u
/// user defined data type
#define PMD_USER_DEFINED_6   0xFFFF0006u
/// user defined data type
#define PMD_USER_DEFINED_7   0xFFFF0007u
/// user defined data type
#define PMD_USER_DEFINED_8   0xFFFF0008u
/// user defined data type
#define PMD_USER_DEFINED_9   0xFFFF0009u
/// user defined data type
#define PMD_USER_DEFINED_10   0xFFFF000au
/// user defined data type
#define PMD_USER_DEFINED_11   0xFFFF000bu
/// user defined data type
#define PMD_USER_DEFINED_12   0xFFFF000cu
/// user defined data type
#define PMD_USER_DEFINED_13   0xFFFF000du
/// user defined data type
#define PMD_USER_DEFINED_14   0xFFFF000eu
/// user defined data type
#define PMD_USER_DEFINED_15   0xFFFF000fu

// pixel origin values
/// The top-right pixel is the first pixel
#define PMD_ORIGIN_TOP_RIGHT       0x00000000u
/// The top-left pixel is the first pixel
#define PMD_ORIGIN_TOP_LEFT        0x00000001u
/// The bottom-right pixel is the first pixel
#define PMD_ORIGIN_BOTTOM_RIGHT    0x00000002u
/// The bottom-left pixel is the first pixel
#define PMD_ORIGIN_BOTTOM_LEFT     0x00000003u

/// The pixel origin is on the right edge of the image
#define PMD_ORIGIN_RIGHT           0x00000000u
/// The pixel origin is on the left edge of the image
#define PMD_ORIGIN_LEFT            0x00000001u
/// The pixel origin is on the top edge of the image
#define PMD_ORIGIN_TOP             0x00000000u
/// The pixel origin is on the bottom edge of the image
#define PMD_ORIGIN_BOTTOM          0x00000002u

// pixel direction values
/// The second pixel is on the right or the left of the first pixel
#define PMD_DIRECTION_HORIZONTAL   0x00000000u
/// The second pixel is aboce or below the first pixel
#define PMD_DIRECTION_VERTICAL     0x00010000u

// sub header types
/// The PMDGenericData structure is used in the PMDDataDescription
#define PMD_GENERIC_DATA           0x00000001u
/// The PMDImageData structure is used in the PMDDataDescription
#define PMD_IMAGE_DATA             0x00000002u

/// @}

/// Generic data description
struct PMDGenericData
{
  /// Specific type of the data
  unsigned subType;
  /// Number of elements in the data
  unsigned numElem;
  /// Size of one element in bytes
  unsigned sizeOfElem;
};

/// Standard PMD image data.
struct PMDImageData
{
  /// Specific type of the data
  unsigned subType;
  /// Number of columns in the image
  unsigned numColumns;
  /// Number of rows in the image
  unsigned numRows;
  /// Number of sub images
  unsigned numSubImages;

  /// Integration times at which the data was captured
  int integrationTime[4];
  /// Modulation frequencies at which the data was captured
  int modulationFrequency[4];
  /// Offsets for up to four separate measurements in mm.
  int offset[4];

  /// Pixel aspect ratio. The ratio is pixelAspectRatio / 1000.0
  int pixelAspectRatio;
  /// Position/direction of the first pixel.
  int pixelOrigin;

  /// Time at which the data was captured. Most significant word.
  int timeStampHi;
  /// Time at which the data was captured. Least significant word.
  unsigned timeStampLo;

  /// Reserved for future use.
  char reserved[24];

  /// Contains user-defined information. This will not be evaluated by the PMDSDK.
  unsigned userData0;
};

/// Layout description of a data block.
struct PMDDataDescription
{
  /// Unique ID of the plugin that generated the data
  unsigned PID;
  /// Unique ID of the data
  unsigned DID;
  /// Interpretation of the data
  unsigned type;
  /// Size of the data block in bytes
  unsigned size;

  /// Type of the sub header
  unsigned subHeaderType;
  /// Specific interpretation-dependent information
  union
    {
      /// Generic data
      struct PMDGenericData gen;
      /// Standard PMD image data
      struct PMDImageData std;
      /// Ensure fixed size
      char fillUpToSizeOfStructure[108];
    }
#ifdef PMD_ANSI_C
      u
#endif
       ;
};

#ifndef PMD_NO_DEPRECATED

typedef struct PMDDataDescription DataDescription;
typedef struct PMDImageData StandardPMDData;
typedef struct PMDGenericData GenericData;
#define GENERIC_DATA                  PMD_GENERIC_DATA
#define STANDARD_PMD_DATA             PMD_IMAGE_DATA

#define RAW_L16_AB        PMD_RAW_L16_AB        
#define RAW_L16_DAB       PMD_RAW_L16_DAB       
#define RAW_B16_DAB    PMD_RAW_B16_DAB    
#define RAW_L16_ABD       PMD_RAW_L16_ABD       
#define RAW_B16_ABD    PMD_RAW_B16_ABD    
#define RAW_P12_D         PMD_RAW_P12_D         
#define RAW_L16_D         PMD_RAW_L16_D         

#define UNKNOWN_DATA   PMD_UNKNOWN_DATA   

#define DISTANCE_LF32       PMD_DISTANCE_LF32       
#define AMPLITUDE_LF32      PMD_AMPLITUDE_LF32      
#define INTENSITY_LF32      PMD_INTENSITY_LF32      
#define REFLECTIVITY_LF32   PMD_REFLECTIVITY_LF32   
#define FLAGS_16            PMD_FLAGS_16            

#define X_COORD_LF32        PMD_X_COORD_LF32        
#define Y_COORD_LF32        PMD_Y_COORD_LF32        
#define Z_COORD_LF32        PMD_Z_COORD_LF32        

#define DISTANCE_I16_AMPLITUDE_I16 PMD_DISTANCE_I16_AMPLITUDE_I16 
#define DISTANCE_100UM_I16_AMPLITUDE_I16 PMD_DISTANCE_100UM_I16_AMPLITUDE_I16 

#define A3_DATA_2_31      PMD_A3_DATA_2_31      
#define A3_DATA_4         PMD_A3_DATA_4         
#define A3_RAWDATA_2_31   PMD_A3_RAWDATA_2_31   
#define A3_DISTANCE       PMD_A3_DISTANCE       
#define A3_AMPLITUDE      PMD_A3_AMPLITUDE      
#define A3_CONFIDENCE     PMD_A3_CONFIDENCE     
#define A3_COORDINATES    PMD_A3_COORDINATES    
#define A3_OBJECT_LIST    PMD_A3_OBJECT_LIST    

#define O3D_DATA_1_7      PMD_O3D_DATA_1_7      
#define O3D_DISTANCE       PMD_O3D_DISTANCE       
#define O3D_INTENSITY     PMD_O3D_INTENSITY     
#define O3D_STDDEV    PMD_O3D_STDDEV    
#define O3D_VIEW_X    PMD_O3D_VIEW_X    
#define O3D_VIEW_Y    PMD_O3D_VIEW_Y    
#define O3D_VIEW_Z    PMD_O3D_VIEW_Z    
#define O3D_X    PMD_O3D_X    
#define O3D_Y    PMD_O3D_Y    
#define O3D_Z    PMD_O3D_Z    

#define USER_DEFINED_0   PMD_USER_DEFINED_0   
#define USER_DEFINED_1   PMD_USER_DEFINED_1   
#define USER_DEFINED_2   PMD_USER_DEFINED_2   
#define USER_DEFINED_3   PMD_USER_DEFINED_3   
#define USER_DEFINED_4   PMD_USER_DEFINED_4   
#define USER_DEFINED_5   PMD_USER_DEFINED_5   
#define USER_DEFINED_6   PMD_USER_DEFINED_6   
#define USER_DEFINED_7   PMD_USER_DEFINED_7   
#define USER_DEFINED_8   PMD_USER_DEFINED_8   
#define USER_DEFINED_9   PMD_USER_DEFINED_9   
#define USER_DEFINED_10   PMD_USER_DEFINED_10   
#define USER_DEFINED_11   PMD_USER_DEFINED_11   
#define USER_DEFINED_12   PMD_USER_DEFINED_12   
#define USER_DEFINED_13   PMD_USER_DEFINED_13   
#define USER_DEFINED_14   PMD_USER_DEFINED_14   
#define USER_DEFINED_15   PMD_USER_DEFINED_15   

#define ORIGIN_TOP_RIGHT       PMD_ORIGIN_TOP_RIGHT       
#define ORIGIN_TOP_LEFT        PMD_ORIGIN_TOP_LEFT        
#define ORIGIN_BOTTOM_RIGHT    PMD_ORIGIN_BOTTOM_RIGHT    
#define ORIGIN_BOTTOM_LEFT     PMD_ORIGIN_BOTTOM_LEFT     

#define ORIGIN_RIGHT           PMD_ORIGIN_RIGHT           
#define ORIGIN_LEFT            PMD_ORIGIN_LEFT            
#define ORIGIN_TOP             PMD_ORIGIN_TOP             
#define ORIGIN_BOTTOM          PMD_ORIGIN_BOTTOM          

#define DIRECTION_HORIZONTAL   PMD_DIRECTION_HORIZONTAL   
#define DIRECTION_VERTICAL     PMD_DIRECTION_VERTICAL     

#endif

END_EXTERN_C

/// @}

#endif 
