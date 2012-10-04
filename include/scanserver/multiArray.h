/**
 * @file
 * @brief 
 *
 * @author Thomas Escher
 */

#ifndef MULTI_ARRAY_H
#define MULTI_ARRAY_H

#include <iostream>
using std::cout;
using std::cerr;
using std::endl;

#include "scanserver/cache/cacheDataAccess.h"



/**
 * @brief Point-Array imitation for xyz[i][0..2] calls
 */
template<typename return_type>
class TripleArray : public CacheDataAccess {
public:
  //! Take ownership of a cache data
  TripleArray(CacheDataAccess&& cd) : CacheDataAccess(cd) {}

  //! Move constructor of this type like CacheData, transfer lock
  TripleArray(TripleArray&& other) : CacheDataAccess(other) {}

  //! Represent the CacheData as an array of T[3]'s
  inline return_type* operator[](unsigned int i) const
  {
    return reinterpret_cast<return_type*>(getData()) + (i*3);
  }
  
  //! Count of T[3] objects in this CacheData
  inline unsigned int size() const { return CacheDataAccess::getSize() / (3*sizeof(return_type)); }
};

/**
 * @brief Point-Array imitation for value[i] calls
 */
template<typename return_type>
class SingleArray : public CacheDataAccess {
public:
  //! Take ownership of a cache data
  SingleArray(CacheDataAccess&& cd) : CacheDataAccess(cd) {}

  //! Move constructor of this type like CacheData, transfer lock
  SingleArray(SingleArray&& other) : CacheDataAccess(other) {}

  //! Represent the CacheData as an array of T's
  inline return_type& operator[](unsigned int i) const
  {
    return *(reinterpret_cast<return_type*>(getData()) + i);
  }
  
  //! Count of T objects in this CacheData
  inline unsigned int size() const { return CacheDataAccess::getSize() / sizeof(return_type); }
};

typedef TripleArray<double> DataXYZ;
typedef TripleArray<unsigned char> DataRGB;
typedef SingleArray<float> DataReflectance;
typedef SingleArray<float> DataTemperature;
typedef SingleArray<float> DataAmplitude;
typedef SingleArray<int> DataType;
typedef SingleArray<float> DataDeviation;

/**
 * To simplify T** access patterns for an array of T[3] (points), this RAII type class helps creating and managing this pointer array on the stack.
 */
template<typename T>
class Array {
public:
  //! Create a temporary array and fill it sequentially with pointers to each point
  Array(const TripleArray<T>& data) {
    unsigned int size = data.size();
    m_array = new T*[size];
    for(unsigned int i = 0; i < size; ++i)
      m_array[i] = data[i];
  }
  
  //! Removes the temporary array on destruction (RAII)
  ~Array() {
    delete[] m_array;
  }
  
  //! Conversion operator to interface the MultiArray to a T** array
  inline T* const* get() const { return m_array; }
private:
  T** m_array;
};

#endif //MULTI_ARRAY_H
