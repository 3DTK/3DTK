/*
 * cacheHandler implementation
 *
 * Copyright (C) Thomas Escher, Kai Lingemann
 *
 * Released under the GPL version 3.
 *
 */

#include "scanserver/cache/cacheHandler.h"

#include "scanserver/cache/cacheObject.h"

CacheHandler::CacheHandler(CacheObject* object, CacheManager* manager) :
  m_manager(manager), m_object(object)
{
}

CacheHandler::~CacheHandler()
{
}
