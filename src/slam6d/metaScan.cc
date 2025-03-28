/*
 * metaScan implementation
 *
 * Copyright (C) by the 3DTK contributors
 *
 * Released under the GPL version 3.
 *
 */

/**
 * @file
 * @brief metascan is a collection of scans whcih can be treated just as
 *        a single scan
 *
 * @author Andreas Nuechter. Jacobs University Bremen gGmbH, Germany
 * @author Kai Lingemann. Inst. of CS, University of Osnabrueck, Germany.
 * @author Thomas Escher. Inst. of CS, University of Osnabrueck, Germany.
 */

#include "slam6d/metaScan.h"
#include "slam6d/kdMeta.h"

#ifdef WITH_METRICS
#include "slam6d/metrics.h"
#endif

MetaScan::MetaScan(std::vector<Scan*> scans, int nns_method) : m_scans(scans)
{
  // add this to the global vector for addFrame reasons
  Scan::allScans.push_back(this);
  m_pairs = *(new std::map<std::string, std::pair<unsigned char*, size_t> >);
}

MetaScan::~MetaScan()
{
  // remove this from the global vector for addFrame reasons
  for(ScanVector::iterator it = Scan::allScans.begin();
      it != Scan::allScans.end();
      ++it) {
    if(*it == this) {
      Scan::allScans.erase(it);
      break;
    }
  }
}

void MetaScan::createSearchTreePrivate()
{
#ifdef WITH_METRICS
  Timer tc = ClientMetric::create_metatree_time.start();
#endif //WITH_METRICS

  // TODO: there is no nns_type switch option for this one
  // because no reduced points are copied, this could be
  // implemented if e.g. cuda is required on metascans
  kd = new KDtreeMetaManaged(m_scans);

#ifdef WITH_METRICS
  ClientMetric::create_metatree_time.end(tc);
#endif //WITH_METRICS
}

size_t MetaScan::size() const
{
  return m_scans.size();
}

Scan* MetaScan::getScan(size_t i) const
{
  return m_scans.at(i);
}
