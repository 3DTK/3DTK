/*
 * tscanserver implementation
 *
 * Copyright (C) Thomas Escher, Kai Lingemann
 *
 * Released under the GPL version 3.
 *
 */

/**
 * @file
 * @brief Dynamic management of scans.
 * @author Thomas Escher
 */

#include <iostream>
using std::cout;
using std::cerr;
using std::endl;
#include <string>
using std::string;

// for signals
#include <csignal>
// for getopt
#ifndef _MSC_VER
#include <getopt.h>
#else
#include "XGetopt.h"
#endif

#include "scanserver/serverInterface.h"
#include "scanserver/cacheIO.h"
#include "scanserver/scanHandler.h"



bool keep_temp_files = false;



void signal_segv(int v)
{
  static sig_atomic_t signal_once = false;
  if(!signal_once) {
    signal_once = true;
    cout << endl
         << "# Scanserver closed due to segmentation fault #" << endl;
    
    // remove server and memory
    ServerInterface::destroy();
  
    // clean up temporary files
    if(!keep_temp_files)
      CacheIO::removeTemporaryDirectory();
  }
  exit(-1);
}

void signal_interrupt(int v)
{
  static sig_atomic_t signal_once = false;
  if(!signal_once) {
    signal_once = true;
    cout << endl
         << "# Scanserver closed #" << endl;
    
    // remove server and memory
    ServerInterface::destroy();
  
    // clean up temporary files
    if(!keep_temp_files)
      CacheIO::removeTemporaryDirectory();
  }
  exit(-1);
}



void usage(const char* name)
{
#ifndef _MSC_VER
  const string bold("\033[1m");
  const string normal("\033[m");
#else
  const string bold("");
  const string normal("");
#endif
  cout <<bold<< "USAGE" <<normal<< endl
    << "   " << name << " [options]" << endl << endl
    <<bold<< "OPTIONS" <<normal<< endl
    << "  "<<bold<<"-c"<<normal<<" NR, "<<bold<<"--cachesize"<<normal<<" NR   [default 1500]" << endl
    << "        Size of shared memory for cache objects in MB. Increase for less reloading of scans and reduced points." << endl
    << "  "<<bold<<"-d"<<normal<<" NR, "<<bold<<"--datasize"<<normal<<" NR   [default 150]" << endl
    << "        Size of shared memory for main data structures in MB. Increase for huge amounts of scans." << endl
    << "  "<<bold<<"-b"<<normal<<" 0/1, "<<bold<<"--binary_scan_cache"<<normal<<"   [default on]" << endl
    << "        Save scans in a binary representation if removed from memory for faster reloading." << endl
    << "        Useful for trying different range or reduction parameters, but will use much space." << endl
    << "  "<<bold<<"-t"<<normal<<" path, "<<bold<<"--temporary_path"<<normal<<" path   [default temp]" << endl
    << "        Directory for holding temporary cache object files." << endl
/*
    << "  "<<bold<<"-k"<<normal<<", "<<bold<<"--keep"<<normal<<"   [default off]" << endl
    << "        Keep temporary cache objects after server is shut down."<<" Not implemented!" << endl
*/
  ;
}

void parseArgs(int argc, char** argv, std::size_t& cache_size, std::size_t& data_size, string& temporary_path, bool& keep, bool& binary_scan_cache)
{
  int  c;
  extern char *optarg;
  
  static struct option longopts[] = {
    {"cachesize", required_argument, 0, 'c'},
    {"datasize", required_argument, 0, 'd'},
    {"temporary_path", required_argument, 0, 't'},
    {"keep", no_argument, 0, 'k'},
    {"binary_scan_cache", required_argument, 0, 'b'},
    {"help", no_argument, 0, '?'}
  };
  
  while((c = getopt_long(argc, argv, "c:d:t:b:k?", longopts, 0)) != -1) {
    switch(c) {
      case 'c':
        cache_size = atoi(optarg);
        break;
      case 'd':
        data_size = atoi(optarg);
        break;
      case 't':
        temporary_path = optarg;
        break;
      case 'k':
/* no parameter-tco database yet implemented
        keep = true;
*/
        break;
      case 'b':
        binary_scan_cache = (atoi(optarg)==0? false: true);
        break;
      case '?':
        usage(argv[0]);
        exit(0);
      default:
        exit(1);
    }
  }
}

int main(int argc, char** argv)
{
  // check for singleton process
  // TODO
  
  // default parameters
  std::size_t cache_size = 750;
  std::size_t data_size = 75;
//  std::size_t cache_size = 150;
//  std::size_t data_size = 15;
  string temporary_path = "temp";
  bool binary_scan_cache = true;
  
  // parse arguments
  parseArgs(argc, argv, cache_size, data_size, temporary_path, keep_temp_files, binary_scan_cache);
  
  // create temporary directory and configure ScanHandler if so desired
  CacheIO::createTemporaryDirectory(temporary_path);
  if(binary_scan_cache)
    ScanHandler::setBinaryCaching();
  
  // create the server instance
  cout << "Starting scanserver." << endl
    << "  Cache size: " << cache_size << "MB, Data Size: " << data_size << "MB." << endl
    << "  Binary scan caching: " << (binary_scan_cache? "yes": "no") << endl;
  ServerInterface* server = ServerInterface::create(data_size*1024*1024, cache_size*1024*1024);
  cout << endl;
  
  // prepare signal handlers after server is created
  signal(SIGSEGV, signal_segv);
  signal(SIGINT,  signal_interrupt);
  signal(SIGTERM, signal_interrupt);
  
  // run forrest, run
  server->run();
  
  // end of line!
  cout << "Stopping scanserver." << endl;
  ServerInterface::destroy();
  
  // clean up temporary files
  if(!keep_temp_files)
    CacheIO::removeTemporaryDirectory();
}
