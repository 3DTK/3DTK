/**
 * @file
 * @brief Metrics for measuring various aspects of an application
 * @author Thomas Escher
 */

#ifndef METRICS_H
#define METRICS_H

#include <vector>

#ifdef _MSC_VER
#include <windows.h>
#else
#include <sys/time.h>
#endif

template<typename T>
class Metric {
public:
  //! Reserve a minimum of 1k values to reduced allocation overhead in between
  Metric(unsigned int reserve = 1000) {
    m_values.reserve(reserve);
  }

  //! Print the sum of this metric
  T sum() const {
    T value = 0;
    for(typename std::vector<T>::const_iterator it = m_values.begin(); it != m_values.end(); ++it)
      value += *it;
    return value;
  }

  //! Print the average of this metric
  T average() const {
    if(m_values.size() != 0)
      return sum() / (T)(m_values.size());
    else
      return (T)0;
  }
  
  std::size_t size() const {
    return m_values.size();
  }
  
  void reset() {
    m_values.clear();
  }

protected:
  inline void commit(const T& value) {
    m_values.push_back(value);
  }

private:
  std::vector<T> m_values;
};

#ifdef _MSC_VER
  typedef LARGE_INTEGER Timer;
#else
  typedef timeval Timer;
#endif

/**
 * @brief Measures time differences in seconds.
 */
class TimeMetric : public Metric<double> {
public:
  TimeMetric(unsigned int reserve = 1000);

  //! Start the timer
  Timer start();

  //! End the timer and commit value
  void end(Timer&);

private:
#ifdef _MSC_VER
  static LARGE_INTEGER frequency;
  static bool init;
#endif
};

/**
 * @brief Measures access count or size.
 */
class CounterMetric : public Metric<unsigned long long> {
public:
  CounterMetric(unsigned int reserve = 1000);
  void add(unsigned long long value = 1);

private:
};



struct ServerMetric {
  static TimeMetric scan_loading, cacheio_write_time, cacheio_read_time;
  static CounterMetric cacheio_write_size, cacheio_read_size;
  static void print();
};

struct ClientMetric {
  static TimeMetric
    // Scan
    // directory loading
    read_scan_time, scan_load_time,
    // reduce, transform, copy, create tree as part of the reduction and tree preparation
    calc_reduced_points_time, transform_time, copy_original_time, create_tree_time,
    // new scanserver only times
    on_demand_reduction_time, create_metatree_time,
    // part of transform dealing with frames
    add_frames_time,
    // slam6D
    matching_time,
    // ClientInterface
    clientinterface_time, cache_miss_time, allocate_time, frames_time;
  static void print();
};

#endif //METRICS_H
