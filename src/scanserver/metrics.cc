/*
 * metrics implementation
 *
 * Copyright (C) Thomas Escher, Kai Lingemann
 *
 * Released under the GPL version 3.
 *
 */

#include "scanserver/metrics.h"

#ifdef _MSC_VER
LARGE_INTEGER TimeMetric::frequency;
bool TimeMetric::init = false;
#endif

TimeMetric::TimeMetric(unsigned int reserve) :
  Metric(reserve)
{
#ifdef _MSC_VER
  if(!TimeMetric::init) {
    QueryPerformanceFrequency(&TimeMetric::frequency);
    TimeMetric::init = true;
  }
#endif
}

Timer TimeMetric::start()
{
  Timer start;
#ifdef _MSC_VER
  QueryPerformanceCounter(&start);
#else
  gettimeofday(&start, 0);
#endif
  return start;
}

void TimeMetric::end(Timer& start)
{
  double delta;
#ifdef _MSC_VER
  LARGE_INTEGER end;
  QueryPerformanceCounter(&end);
  delta = (double)(end.QuadPart - start.QuadPart) / frequency.QuadPart;
#else
  timeval end;
  gettimeofday(&end, 0);
  delta = (double)(end.tv_sec - start.tv_sec)
    + (double)(end.tv_usec - start.tv_usec) / 1000000.0;
#endif
  commit(delta);
}

CounterMetric::CounterMetric(unsigned int reserve) :
  Metric(reserve)
{
}

void CounterMetric::add(unsigned long long value)
{
  commit(value);
}



#include <iostream>
using std::cout;
using std::endl;

TimeMetric ServerMetric::scan_loading, ServerMetric::cacheio_write_time, ServerMetric::cacheio_read_time;
CounterMetric ServerMetric::cacheio_write_size, ServerMetric::cacheio_read_size;

TimeMetric
  ClientMetric::read_scan_time,
  ClientMetric::scan_load_time,
  ClientMetric::calc_reduced_points_time,
  ClientMetric::transform_time(100000), // avoid resizing in multithreaded transform calls by graph-slam
  ClientMetric::copy_original_time,
  ClientMetric::create_tree_time,
  ClientMetric::on_demand_reduction_time,
  ClientMetric::create_metatree_time,
  ClientMetric::add_frames_time,
  ClientMetric::matching_time,
  ClientMetric::clientinterface_time,
  ClientMetric::cache_miss_time,
  ClientMetric::allocate_time,
  ClientMetric::frames_time;

void printTime(const TimeMetric& m, unsigned int indentation = 1)
{
  for(unsigned int i = 1; i <= indentation; ++i) {
    cout << "  ";
  }
  cout << m.sum() << "s";
  if(m.size() != 1) {
    cout << " (" << m.average() << "s average of " << m.size() << " calls)";
  }
  cout << endl;
}

void ServerMetric::print()
{
  cout << "= Metric server information =" << endl
    << "Time spent for loading scans (in ScanHandler::load):" << endl
    << "  Amount: " << scan_loading.size() << endl
    << "  Time: " << scan_loading.sum() << "s (" << scan_loading.average() << "s avg.)" << endl
    << endl
    << "CacheIO reads:" << endl
    << "  Amount: " << cacheio_read_size.size() << endl
    << "  Size: " << cacheio_read_size.sum()/1024/1024 << "MB (" << cacheio_read_size.average()/1024 << "KB avg.)" << endl
    << "  Time: " << cacheio_read_time.sum() << "s (" << cacheio_read_time.average() << "s avg.)" << endl
    << endl
    << "CacheIO writes:" << endl
    << "  Amount: " << cacheio_write_size.size() << endl
    << "  Size: " << cacheio_write_size.sum()/1024/1024 << "MB (" << cacheio_write_size.average()/1024 << "KB avg.)" << endl
    << "  Time: " << cacheio_write_time.sum() << "s (" << cacheio_write_time.average() << "s avg.)" << endl
    << "= Resetting metric information =" << endl
    << endl;
  scan_loading.reset();
  cacheio_write_time.reset();
  cacheio_read_time.reset();
  cacheio_write_size.reset();
  cacheio_read_size.reset();
}

void ClientMetric::print()
{
  cout << "= Metric client information =" << endl;

  // ClientInterface specific ones
#ifdef WITH_SCANSERVER
  cout << "Time for all ClientInterface messages:" << endl;
  printTime(clientinterface_time);
  
  cout << "  [" << endl;
  
  cout << "    Time for cache misses:" << endl;
  printTime(cache_miss_time, 3);
  
  cout << "    Time for cache allocations:" << endl;
  printTime(allocate_time, 3);
  
  // TODO: invalidations too?
  
  cout << "    Time for frame calls:" << endl;
  printTime(frames_time, 3);
  
  cout << "  ]" << endl;
  
  cout << endl;
#endif //WITH_SCANSERVER
  
  // Scan: Loading, reducing and SearchTree related ones
#ifdef WITH_SCANSERVER
  // barebone preparation and parameter setting
  if(read_scan_time.size()) {
    cout << "Time for loading directory:" << endl;
    printTime(read_scan_time);
  }
  
  // getXYZ in calcReducedPoints
  if(scan_load_time.size()) {
    cout << "Time for loading scans:" << endl;
    printTime(scan_load_time);
  }
  
#else //WITH_SCANSERVER
  // load all scans
  if(read_scan_time.size()) {
    cout << "Time for loading directory and scans:" << endl;
    printTime(read_scan_time);
  }
#endif //WITH_SCANSERVER
  
  if(calc_reduced_points_time.size()) {
    cout << "Time for reducing scans:" << endl;
    printTime(calc_reduced_points_time);
  }
  
  // old: red_lum for SearchTree, duplicating all contained reduced points and copying them into red_lum again for MetaScans
  // new: caching reduced points for saving time
  if(copy_original_time.size()) {
    cout << "Time for copying reduced points:" << endl;
    printTime(copy_original_time);
  }
  
  if(create_tree_time.size()) {
    cout << "Time for creating SearchTree:" << endl;
    printTime(create_tree_time);
  }
  
#ifdef WITH_SCANSERVER
  // only for new, replaces copying reduced points from old
  if(create_metatree_time.size()) {
    cout << "Time for creating meta-SearchTree:" << endl;
    printTime(create_metatree_time);
  }
  
  if(on_demand_reduction_time.size()) {
    cout << "Time in on-demand reduction [reduction+transform+copy / copy]:" << endl;
    printTime(on_demand_reduction_time);
  }
#endif //WITH_SCANSERVER
  
  if(transform_time.size()) {
    cout << "Time for transform:" << endl;
    printTime(transform_time);
  }
  
  // SLAM
  if(matching_time.size()) {
    cout << endl;
    cout << "Matching time:" << endl;
    printTime(matching_time);
#ifdef WITH_SCANSERVER
    cout << "Corrected matching time without on-demand loading, reduction and tree creation:" << endl;
    // match - scan loading in calcReducedPoints - calcReducedPoints - transform (irrelevant) - copy to original - createTree
    cout << "  " <<
      matching_time.sum()
      - scan_load_time.sum()
      - calc_reduced_points_time.sum()
      - copy_original_time.sum()
      - create_tree_time.sum()
      << "s" << endl;
#endif //WITH_SCANSERVER
  }
  
  cout << endl;
}
