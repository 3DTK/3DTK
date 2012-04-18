/**
 * @file
 * @brief 
 *
 * @author Thomas Escher
 */

#ifndef CACHE_IO_H
#define CACHE_IO_H

#include <string>

/**
 * @brief Serialization management for binary data, intended for use in CacheHandlers.
 *
 * This class manages the assignment of unique IDs for CacheHandlers to use to identify their files.
 * Data is (de)serialized via read and write calls and existance (if so, the file-/datasize too) can be checked via check.
 * All files are created in a directory given by createTemporaryDirectory which has to be called before and read/writes to function properly. All files are named 'ddddd.tco' starting from zero.
 */
class CacheIO {
public:
  typedef std::string IDType;
  
  //! Create a directory for temporary cache objects to save in
  static void createTemporaryDirectory(std::string& path);

  //! Clean up temporary files
  static void removeTemporaryDirectory();
  
  //! Creates a unique Id to use for these functions
  static IDType getId();

  //! Check if a physical representation of this cache entry exists and returns non-zero size for the data
  static unsigned int check(IDType& id);

  //! Read from file into the data pointer
  static void read(IDType& id, char* data);

  //! Write data into a file represented by id
  static void write(IDType& id, char* data, unsigned int size);
private:
  static std::string path;
  static unsigned int free_id;
};

#endif //CACHE_IO_H
