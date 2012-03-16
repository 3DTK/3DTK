/**
 * @file 
 * @brief allocator object that gets chunks of memory and then hands parts of them to a user
 * @author Jan Elsberg. Automation Group, Jacobs University Bremen gGmbH, Germany. 
 */
#ifndef ALLOCATOR_H
#define ALLOCATOR_H

#include <vector>

using std::vector;

class Allocator {

  vector<unsigned char *> mem;
  const unsigned int chunksize;
  unsigned int index;
  unsigned long int memsize;
  unsigned long int wastedspace;

public:

  Allocator(unsigned int _csize = (1 << 20)) : chunksize(_csize), index(_csize), memsize(0), wastedspace(0) {}
  ~Allocator() {
    for (unsigned int i = 0; i < mem.size(); i++) {
      delete[] mem[i];
    }
  }

  void printSize() {
    cout << "Alloc'd  " << memsize/(1024*1024.0) << " Mb " << endl;
    cout << " wasted  " << wastedspace/(1024*1024.0) << " Mb " << endl;
  }

  template<class T> 
    T *allocate() {
      return allocate<T>(1);
    }
  
  
  template<class T> 
    T *allocate(unsigned int nr) {
      unsigned int size = sizeof(T) * nr;
      unsigned char *chunk;
      if (size + index > chunksize) {
        wastedspace += (chunksize-index);
        if (chunksize > size) {
          chunk = new unsigned char[chunksize];
		  memset(chunk, 0, chunksize);
          memsize+=chunksize;
        } else {
          chunk = new unsigned char[size];
		  memset(chunk, 0, size);
          memsize+=size;
        }
        mem.push_back(chunk);
        index = 0;
      } else {
        chunk = mem.back();
        chunk = chunk + index;
      }

      index += size;                  // increment index
      return (T*)chunk;                 
    }
};


class PackedAllocator {

  vector<unsigned char *> mem;
  vector<unsigned int > index;
  const unsigned int chunksize;
  unsigned long int memsize;

public:

  PackedAllocator(unsigned int _csize = (1 << 20)) : chunksize(_csize) {
    memsize = 0;
  }
  ~PackedAllocator() {
    for (unsigned int i = 0; i < mem.size(); i++) {
      delete[] mem[i];
    }
  }

  void printSize() {
    cout << "Alloc'd  " << memsize/(1024*1024.0) << " Mb " << endl;
    
    unsigned long int wastedspace = 0;
    for (unsigned int i = 0; i < index.size(); i++) {
      if (index[i] < chunksize) {
        wastedspace += chunksize - index[i];
      }
    }
    cout << "wasted  " << wastedspace/(1024*1024.0) << " Mb " << endl;
  }

  template<class T> 
    T *allocate() {
      return allocate<T>(1);
    }
  
  
  template<class T> 
    T *allocate(unsigned int nr) {
      unsigned int size = sizeof(T) * nr;
      unsigned char *chunk;
      for (unsigned int i = 0; i < index.size(); i++) {
        if ( !(size + index[i] > chunksize) ) {
          // found a suitable entry
          chunk = mem[i];
          chunk = chunk + index[i];          // pointer to free byte
          index[i] += size;                  // increment index
          return (T*)chunk;                 
        }
      }
      // no chunk is large enough... make new one
      if (chunksize > size) {
        chunk = new unsigned char[chunksize];
        memset(chunk, 0, chunksize);
        memsize += chunksize;
      } else {   // in case the requested memory is larger than our chunks, make a single chunk thats large enough
        chunk = new unsigned char[size];
        memset(chunk, 0, size);
		memsize += size;
      }
      mem.push_back(chunk);
      index.push_back(size);
      return (T*)chunk;                 
    }
};



#endif
