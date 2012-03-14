/**
 * @file
 * @brief IO for .frames files.
 *
 * @author Thomas Escher
 */

#ifndef SCANSERVER_FRAME_IO_H
#define SCANSERVER_FRAME_IO_H

#include "scanserver/frame.h"



/**
 * @brief This class handles the serialization of .frames files and saves/extracts the contained Frame instances.
 */
class FrameIO
{
public:
  //! Loads a frames file and fills the given shared vector
  static void loadFile(const char* dir, const char* identifier, FrameVector& frames);

  //! Saves a frames file from a shared frames vector
  static void saveFile(const char* dir, const char* identifier, const FrameVector& frames);
};

#endif //SCANSERVER_FRAME_IO_H
