/**
 * @file
 * @brief tools for file format processing
 * @author Andreas Nuechter. Jacobs University Bremen gGmbH, Germany.
 * @author Thomas Escher. Institute of Computer Science, University of Osnabrueck, Germany.
 */

#ifndef __IO_UTILS_H__
#define __IO_UTILS_H__

#include "slam6d/io_types.h"
#include "slam6d/globals.icc"

#include <string>
using std::string;
using std::cerr;

/**
 * Make type, start and end a write once only class for parseFormatFile because
 * we know the directory only after the type, start and end parameters may have
 * been written already, so encapsulate this write once behaviour in this class.
 */
template<typename T>
class WriteOnce {
public:
  WriteOnce(T& value) : value(value), written(false) {}
  WriteOnce& operator=(const T& other) { if(!written) { value = other; written = true; } return *this; }
  operator T() const { return value; }
private:
  T& value;
  bool written;
};

void parseFormatFile(string& dir, WriteOnce<IOType>& type, WriteOnce<int>& start, WriteOnce<int>& end);

#endif
