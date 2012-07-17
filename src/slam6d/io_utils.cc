#include "slam6d/io_utils.h"

#include <fstream>
using std::ifstream;

/**
 * Parsing of a formats file in the scan directory for default type and scan
 * index ranges without overwriting user set parameters. Does nothing if
 * file doesn't exist.
 * 
 * @param dir directory the scans and format file are contained in
 * @param type which ScanIO to use for the scans in that directory
 * @param start index for the first valid scan
 * @param end index for the last valid scan
 */
void parseFormatFile(string& dir, WriteOnce<IOType>& type, WriteOnce<int>& start, WriteOnce<int>& end)
{
  ifstream file((dir+"format").c_str());
  if(!file.good()) return;
  
  string line, key, value, format;
  while(getline(file, line)) {
    size_t pos = line.find('=');
    key = trim(line.substr(0, pos - 0));
    value = trim(line.substr(pos+1));
    if(key == "format") {
      try {
        format = value;
        type = formatname_to_io_type(format.c_str());
      } catch (...) { // runtime_error
        cerr << "Error while parsing format file: Format '" << format << "' unknown." << endl;
        break;
      }
    } else if(key == "start") {
      stringstream str(value.c_str());
      int s;
      str >> s;
      start = s;
    } else if(key == "end") {
      stringstream str(value.c_str());
      int e;
      str >> e;
      end = e;
    } else {
      cerr << "Error while parsing format file: Unknown key '" << key << "'" << endl;
      break;
    }
  }
}
