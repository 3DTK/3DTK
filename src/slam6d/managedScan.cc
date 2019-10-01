/*
 * managedScan implementation
 *
 * Copyright (C) Kai Lingemann, Thomas Escher
 *
 * Released under the GPL version 3.
 *
 */

#include "slam6d/managedScan.h"

#include "scanserver/clientInterface.h"
#include "slam6d/Boctree.h"
#include "slam6d/kdManaged.h"

#ifdef WITH_METRICS
#include "slam6d/metrics.h"
#endif // WITH_METRICS

#include <sstream>

#include <boost/filesystem/operations.hpp>
using namespace boost::filesystem;

SharedScanVector* ManagedScan::shared_scans = 0;

void ManagedScan::openDirectory(const std::string& path,
                                IOType type,
                                int start,
                                int end)
{
  // start the client first
  try {
    ClientInterface::create();
  } catch(std::runtime_error& e) {
    std::cerr << "ClientInterface could not be created: " << e.what() << std::endl;
    std::cerr << "Start the scanserver first." << std::endl;
    exit(-1);
  }

#ifdef WITH_METRICS
  Timer t = ClientMetric::read_scan_time.start();
#endif //WITH_METRICS

  ClientInterface* client = ClientInterface::getInstance();
  shared_scans = client->readDirectory(path.c_str(), type, start, end);

  for(SharedScanVector::iterator it = shared_scans->begin();
      it != shared_scans->end();
      ++it) {
    // add a scan with reference on the shared scan
    SharedScan* shared = it->get();
    ManagedScan* scan = new ManagedScan(shared);
    Scan::allScans.push_back(scan);
  }

#ifdef WITH_METRICS
  ClientMetric::read_scan_time.end(t);
#endif //WITH_METRICS
}

void ManagedScan::closeDirectory()
{
  // clean up the scan vector
  for(std::vector<Scan*>::iterator it = Scan::allScans.begin();
      it != Scan::allScans.end();
      ++it)
    delete *it;
  allScans.clear();
  // remove the shared scan vector
  ClientInterface* client = ClientInterface::getInstance();
#ifdef WITH_METRICS
  ClientInterface::getInstance()->printMetrics();
#endif //WITH_METRICS
  client->closeDirectory(shared_scans);
}

std::size_t ManagedScan::getMemorySize()
{
  ClientInterface* client = ClientInterface::getInstance();
  return client->getCacheSize();
}



ManagedScan::ManagedScan(SharedScan* shared_scan) :
  m_shared_scan(shared_scan),
  m_reduced_ready(false),
  m_reset_frames_on_write(true)
{
  // request pose from the shared scan
  double* euler = m_shared_scan->getPose();
  rPos[0] = euler[0];
  rPos[1] = euler[1];
  rPos[2] = euler[2];
  rPosTheta[0] = euler[3];
  rPosTheta[1] = euler[4];
  rPosTheta[2] = euler[5];

  // write original pose matrix
  EulerToMatrix4(euler, &euler[3], transMatOrg);

  // initialize transform matrices from the original one,
  // could just copy transMatOrg to transMat instead
  transformMatrix(transMatOrg);

  // reset the delta align matrix to represent only the transformations
  // after local-to-global (transMatOrg) one
  M4identity(dalignxf);
}

ManagedScan::~ManagedScan()
{
  // TODO: something to do?
}

void ManagedScan::setRangeFilter(double max, double min)
{
  m_shared_scan->setRangeParameters(max, min);
}

void ManagedScan::setHeightFilter(double top, double bottom)
{
  m_shared_scan->setHeightParameters(top, bottom);
}

void ManagedScan::setCustomFilter(std::string& cFiltStr)
{
  m_shared_scan->setCustomParameters(cFiltStr);
}

void ManagedScan::setRangeMutation(double range)
{
  m_shared_scan->setRangeMutationParameters(range);
}

void ManagedScan::setScaleFilter(double scale)
{
  m_shared_scan->setScaleParameters(scale);
}


void ManagedScan::setReductionParameter(double voxelSize,
                                        int nrpts,
                                        PointType pointtype)
{
  Scan::setReductionParameter(voxelSize, nrpts, pointtype);
  // set parameters to invalidate old cache data
  std::stringstream s;
  s << voxelSize << " " << nrpts << " " << transMatOrg;
  m_shared_scan->setReductionParameters(s.str().c_str());
}

void ManagedScan::setShowReductionParameter(double voxelSize,
                                            int nrpts,
                                            PointType pointtype)
{
  show_reduction_voxelSize = voxelSize;
  show_reduction_nrpts = nrpts;
  show_reduction_pointtype = pointtype;
  // set parameters to invalidate old cache data
  std::stringstream s;
  s << voxelSize << " " << nrpts;
  m_shared_scan->setShowReductionParameters(s.str().c_str());
}

void ManagedScan::setOcttreeParameter(double reduction_voxelSize,
                                      double octtree_voxelSize,
                                      PointType pointtype,
                                      bool loadOct,
                                      bool saveOct,
                                      bool autoOct)
{
  Scan::setOcttreeParameter(reduction_voxelSize,
                            octtree_voxelSize,
                            pointtype,
                            loadOct,
                            saveOct,
                            autoOct);
  // set octtree parameters to invalidate cached ones with other
  // parameters (changing range/height is already handled)
  std::stringstream s;
  s << reduction_voxelSize << " "
    << octtree_voxelSize << " "
    << pointtype.toFlags();
  m_shared_scan->setOcttreeParameters(s.str().c_str());
}

DataPointer ManagedScan::get(const std::string& identifier)
{
  if(identifier == "xyz") {
    return m_shared_scan->getXYZ();
  } else
    if(identifier == "rgb") {
      return m_shared_scan->getRGB();
    } else
      if(identifier == "reflectance") {
        return m_shared_scan->getReflectance();
      } else
        if(identifier == "temperature") {
          return m_shared_scan->getTemperature();
        } else
          if(identifier == "amplitude") {
            return m_shared_scan->getAmplitude();
          } else
            if(identifier == "type") {
              return m_shared_scan->getType();
            } else
              if(identifier == "deviation") {
                return m_shared_scan->getDeviation();
              } else
                if(identifier == "xyz reduced") {
                  // if this is a fresh run, initialize reduced
                  // properly via original or creating it anew
                  if(!m_reduced_ready) {
                    calcReducedOnDemand();
                  }
                  return m_shared_scan->getXYZReduced();
                } else
                  if(identifier == "xyz reduced original") {
                    // if reduction has completed, original will exist
                    // (either from last run or created in this run)
                    if(!m_reduced_ready) {
                      calcReducedOnDemand();
                    }
                    return m_shared_scan->getXYZReducedOriginal();
                  } else
                    if(identifier == "xyz reduced show") {
                      if(m_shared_scan->getXYZReducedShow().valid())
                        return m_shared_scan->getXYZReducedShow();
                      calcReducedShow();
                      return m_shared_scan->getXYZReducedShow();
                    } else
                      if(identifier == "octtree") {
                        if(m_shared_scan->getOcttree().valid())
                          return m_shared_scan->getOcttree();
                        createOcttree();
                        return m_shared_scan->getOcttree();
                      } else
                        if(identifier == "normal reduced") {
                          std::cout << "FIXME Upgrade SharedScan for the normal reduced data field!"
                               << std::endl;
                          return DataPointer(0,0);
                        }
  {
    throw std::runtime_error(std::string("Identifier '") + identifier
                        + "' not compatible with ManagedScan::get. "
                        + "Upgrade SharedScan for this data field.");
  }
}

void ManagedScan::get(IODataType types)
{
  m_shared_scan->prefetch(types);
}

DataPointer ManagedScan::create(const std::string& identifier, size_t size)
{
  // map identifiers to functions in SharedScan and scale back size
  // from bytes to number of points
  if(identifier == "xyz reduced") {
    return m_shared_scan->createXYZReduced(size / (3*sizeof(double)));
  } else
    if(identifier == "xyz reduced original") {
      return m_shared_scan->createXYZReducedOriginal(size / (3*sizeof(double)));
    } else
      if(identifier == "reflectance") {
        return m_shared_scan->createReflectance(size / (1*sizeof(double)));
      } else {
        throw std::runtime_error(std::string("Identifier '") + identifier
                            + "' not compatible with ManagedScan::create. "
                            + "Upgrade SharedScan for this data field.");
        }
}

void ManagedScan::clear(const std::string& identifier)
{
  // nothing to do here
  // TODO: mark CacheObjects with a low priority for
  // faster removal by the manager
}

void ManagedScan::createSearchTreePrivate()
{
  switch(searchtree_nnstype)
    {
    case simpleKD:
      kd = new KDtreeManaged(this);
      break;
    case BOCTree:
      kd = new BOctTree<double>
        (PointerArray<double>(get("xyz reduced original")).get(),
         size<DataXYZ>("xyz reduced original"),
         10.0, PointType(), true);
      break;
    case -1:
      throw std::runtime_error("Cannot create a SearchTree without setting a type.");
    default:
      throw std::runtime_error("SearchTree type not implemented for ManagedScan");
    }

  // TODO: look into CUDA compability
}

void ManagedScan::calcReducedOnDemandPrivate()
{
  // either copy from original or create them like BasicScan
  DataXYZ xyz_orig(m_shared_scan->getXYZReducedOriginal());
  if(xyz_orig.valid()) {
    // set true to inform further get("xyz reduced original") calls
    // to get the data instead of looping calcReducedOnDemand
    m_reduced_ready = true;
    copyOriginalToReduced();
  } else {
    // create reduced points and transform to initial position,
    // save a copy of this for SearchTree
    calcReducedPoints();
    // set true to inform further get("xyz reduced") calls
    // to get the data instead of looping calcReducedOnDemand
    m_reduced_ready = true;
    transformReduced(transMatOrg);
    copyReducedToOriginal();
  }
}

void ManagedScan::calcReducedShow()
{
  // create an octtree reduction from full points
  DataXYZ xyz(get("xyz"));
  BOctTree<double>* oct = new BOctTree<double>(PointerArray<double>(xyz).get(),
                                               xyz.size(),
                                               show_reduction_voxelSize);

  std::vector<double*> center;
  center.clear();

  if(show_reduction_nrpts > 0) {
    if(show_reduction_nrpts == 1) {
      oct->GetOctTreeRandom(center);
    } else {
		// FIXME: add option to pass true to GetOctTreeRandom()
		//        in the past this was done by letting reduction_nrpts be
		//        negative but ultimately this should be an extra variable
      oct->GetOctTreeRandom(center, show_reduction_nrpts, false);
    }
  } else {
    oct->GetOctTreeCenter(center);
  }

  size_t size = center.size();
  TripleArray<float> xyz_r(m_shared_scan->createXYZReducedShow(size));
  for(size_t i = 0; i < size; ++i) {
    for(size_t j = 0; j < 3; ++j) {
      xyz_r[i][j] = center[i][j];
    }
  }

  delete oct;
}

void ManagedScan::createOcttree()
{
  std::string scanFileName = std::string(m_shared_scan->getDirPath())
    + "scan"
    + getIdentifier()
    + ".oct";
  BOctTree<float>* btree = 0;

  // if loadOct is given, load the octtree under the blind
  // assumption that parameters match
  if(octtree_loadOct && exists(scanFileName)) {
    btree = new BOctTree<float>(scanFileName);
  } else {
    if(octtree_reduction_voxelSize > 0) { // with reduction, only xyz points
      TripleArray<float> xyz_r(get("xyz reduced show"));
      btree = new BOctTree<float>(PointerArray<float>(xyz_r).get(),
                                  xyz_r.size(),
                                  octtree_voxelSize,
                                  octtree_pointtype,
                                  true);
    } else { // without reduction, xyz + attribute points
      float** pts = octtree_pointtype.createPointArray<float>(this);
      size_t nrpts = size<DataXYZ>("xyz");
      btree = new BOctTree<float>(pts,
                                  nrpts,
                                  octtree_voxelSize,
                                  octtree_pointtype,
                                  true);
      for(size_t i = 0; i < nrpts; ++i) delete[] pts[i]; delete[] pts;
    }
    // save created octtree
    if(octtree_saveOct) {
      std::cout << "Saving octree " << scanFileName << std::endl;
      btree->serialize(scanFileName);
    }
  }

  // copy tree into cache
  try {
    size_t size = btree->getMemorySize();
    unsigned char* mem_ptr
      = m_shared_scan->createOcttree(size).get_raw_pointer();
    new(mem_ptr) BOctTree<float>(*btree, mem_ptr, size);
    delete btree; btree = 0;
  } catch(std::runtime_error& e) {
    // delete tree if copy to cache failed
    delete btree;
    throw e;
  }
}


size_t ManagedScan::readFrames()
{
  // automatically read on getFrames
  return m_shared_scan->getFrames().size();
}

void ManagedScan::saveFrames(bool append)
{
  m_shared_scan->saveFrames(append);
}

size_t ManagedScan::getFrameCount()
{
  return m_shared_scan->getFrames().size();
}

void ManagedScan::getFrame(size_t i,
                           const double*& pose_matrix,
                           AlgoType& type)
{
  const Frame& frame(m_shared_scan->getFrames().at(i));
  pose_matrix = frame.transformation;
  type = static_cast<AlgoType>(frame.type);
}

void ManagedScan::addFrame(AlgoType type)
{
  if(m_reset_frames_on_write) {
    m_shared_scan->clearFrames();
    m_reset_frames_on_write = false;
  }
  m_shared_scan->addFrame(transMat, static_cast<unsigned int>(type));
}
