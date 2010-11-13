/** 
 * @file 
 * @brief Representation of the cache for cached k-d tree search.
 * @author Kai Lingemann. Institute of Computer Science, University of Osnabrueck, Germany.
 * @author Andreas Nuechter. Institute of Computer Science, University of Osnabrueck, Germany.
 */

#ifndef __KD_CACHE_H__
#define __KD_CACHE_H__

#include "searchCache.h"
#include "kdparams.h"

// just a prototype
class KDtree_cache;

/**
 * @brief cache item
 * 
 * The return value of the cached k-d tree
 * is a bundle of the closest point and
 * a pointer to the leaf.
 **/
class KDCacheItem : public SearchTreeCacheItem {
public:
  KDCacheItem() { node = 0; };
  KDParams param;
  KDtree_cache *node;
};

/**
 * @brief cache 
 * 
 * The cache consists of an array of KDCacheItems
 * and two Scan numbers.
 **/
class KDCache {
public:
  KDCache() { SourceScanNr = TargetScanNr = 0; item = 0; };
  KDCacheItem *item;  // array of items
  unsigned int SourceScanNr;
  unsigned int TargetScanNr;
};

#endif


