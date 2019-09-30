#ifndef __SCAN_SETTINGS_H__
#define __SCAN_SETTINGS_H__

#include "slam6d/globals.icc"
#include "slam6d/io_types.h"
#include "slam6d/point_type.h"

#include "parsers/range_set_parser.h"

#include <string>
#include <vector>
#include <limits>
#include <algorithm>

template<class T> class range_iterator;

template<class T>
struct range
{
  T min, max;
  size_t clustersize = 1;
  size_t stepsize = 1;
  size_t offset = 0;

  range(const T& a, const T&b, size_t cs = 1, size_t ss = 1)
    : min(a), max(b), clustersize(cs), stepsize(ss)
  {
    if (max == -1) {
      max = UNLIMITED;
    }
  }
  range() : range(0, UNLIMITED) {}

  static const T UNLIMITED;
  bool isLimited() { return max != UNLIMITED && max != -1; }

  static const size_t CLUSTER_ALL = 0;
  void setMerged() { clustersize = CLUSTER_ALL; }
  bool isMerged() { return clustersize == CLUSTER_ALL; }

  std::string toString(int id_len = 3) {
    std::string identifier = to_string(min, id_len);
    if(max != min)
      identifier += ":" + (isLimited() ? to_string(max, id_len) : "-1");
    if (clustersize != 1)
      identifier += "[" + (isMerged() ? "-1" : to_string(clustersize)) + "]";
    if (stepsize != 1)
      identifier += ":" + to_string(stepsize);
    return identifier;
  }
  bool isValid() {
    return !isLimited() || max >= min;
  }
  bool setMaxLimit(T new_max) {
    if (new_max == -1) new_max = UNLIMITED;
    if (max > new_max || !isLimited()) max = new_max;
    return isValid();
  }
  bool setMinLimit(T new_min) {
    if(min < new_min) min = new_min;
    return isValid();
  }
  bool setLimits(T new_min, T new_max) {
    return setMinLimit(new_min) && setMaxLimit(new_max);
  }

  typedef T type;
  typedef range_parser<T, std::string::const_iterator> parser;
  using iterator = range_iterator<T>;
  using const_iterator = range_iterator<const T>;
  iterator begin() { return iterator(this); }
  iterator end() { return iterator(); }
  const_iterator begin() const { return iterator(this); }
  const_iterator end() const { return { iterator() }; }
};
template<class T> const T range<T>::UNLIMITED = std::numeric_limits<T>::max();

template<class T> class range_iterator
{
public:
  static const T END_POS;
  range_iterator() : ref(nullptr), pos(END_POS) {}
  range_iterator(range<T>* r)
    : ref(r), pos(r->min + r->offset), cluster_count(0)
  {
    cluster_start = pos;
  }
  bool operator==(const range_iterator &ri) const {
    return ri.ref == ref && ri.pos == pos;
  }
  bool operator!=(const range_iterator &ri) const {
    return !(*this == ri);
  }
  bool operator<(const range_iterator &ri) const {
    return (ri.ref == ref && pos < ri.pos) || (ri.ref == nullptr && (!ref->isLimited() || pos < ref->max));
  }
  range_iterator<T>& operator+=(int steps) {
    cluster_count += steps;
    if(ref->stepsize == 1 || ref->isMerged() || cluster_count < ref->clustersize) {
      pos += steps;
    }
    else {
      int cluster_jumps = cluster_count / ref->clustersize;
      steps = steps % ref->clustersize;
      pos += cluster_jumps * (ref->stepsize * ref->clustersize) + steps; // * ref->stepsize;
    }
    //NOTE: ref->max needs to be in brackets due to bug in gcc in ubuntu xenial
    // see: https://gcc.gnu.org/ml/gcc-help/2016-01/msg00087.html
    if (ref->isLimited() && (ref->max) < pos)
      pos = END_POS;
    return *this;
  }
  range_iterator<T>& operator++() { return (*this) += 1; }
  range_iterator<T> operator++(int) { auto self = *this; ++(*this); return self; }
  T& operator*() { return pos; }
  bool done() const { return pos == END_POS || !cluster().isValid(); } // && *this < ref->end(); }
  bool clusterDone() const { return done() || (!ref->isMerged() && cluster_count >= ref->clustersize); }
  range_iterator<T>& nextCluster() {
    if (ref->isMerged()) {
      pos = END_POS;
      return *this;
    }
    cluster_start += ref->stepsize * ref->clustersize;
    cluster_count = 0;
    pos = cluster_start;
    if (ref->isLimited() && (ref->max) < pos)
      pos = END_POS;
    return *this;
  }
  range<T> cluster() const {
    T cluster_max = (ref->isMerged() ? ref->max : cluster_start + (ref->clustersize - 1));
    if (ref->isLimited() && cluster_max > ref->max) cluster_max = ref->max;
    return range<T>(cluster_start, cluster_max);
  }

private:
  range<T>* ref;
  T pos;
  size_t cluster_count;
  T cluster_start;
};
template<class T> const T range_iterator<T>::END_POS = std::numeric_limits<T>::max();

template<class RangeType> class multi_range_iterator;

template<class RangeType> struct multi_range
{
  std::vector<RangeType> ranges{};
  bool merged = false;

  multi_range(std::vector<RangeType> rgs, bool merged = false)
    : ranges(rgs), merged(merged)
  {
    if (ranges.empty())
      ranges = { RangeType() };
  }
  multi_range()
    : ranges({ RangeType() }), merged(false)
  {}

  void set(std::string mr_def) {
    parse_multi_range(mr_def, *this);
  }

  typedef typename RangeType::type type;
  typedef multi_range_parser<RangeType, std::string::const_iterator> parser;
  typedef multi_range_iterator<RangeType> iterator;
  typedef multi_range_iterator<const RangeType> const_iterator;
  iterator begin() { return iterator(this); }
  iterator end() { return iterator(); }
  const_iterator begin() const { return iterator(this); }
  const_iterator end() const { return { iterator() }; }
  std::string toString(int id_len = 3) {
    if (ranges.empty()) return "";
    auto it = ranges.begin();
    std::string identifier = it->toString();
    for (++it; it != ranges.end(); ++it)
      identifier += "+" + it->toString(id_len);
    return identifier;
  }
  bool isValid() {
    for (auto it = ranges.begin(); it != ranges.end(); ++it)
      if (!it->isValid()) return false;
    return true;
  }
  bool setMaxLimit(typename RangeType::type new_max) {
    ranges.erase(std::remove_if(ranges.begin(), ranges.end(), [new_max, this](RangeType& elem) -> bool { return !elem.setMaxLimit(new_max); }), ranges.end());
    return ranges.size();
  }
  bool setMinLimit(typename RangeType::type new_min) {
    ranges.erase(std::remove_if(ranges.begin(), ranges.end(), [new_min, this](RangeType& elem) -> bool { return !elem.setMinLimit(new_min); }), ranges.end());
    return ranges.size();
  }
  bool setLimits(typename RangeType::type new_min, typename RangeType::type new_max) {
    return setMinLimit(new_min) && setMaxLimit(new_max);
  }
};
typedef multi_range<multi_range<range<int> > > multi_range_set;

template<class RangeType> class multi_range_iterator
{
public:
  multi_range_iterator() : ref(nullptr) {}
  multi_range_iterator(multi_range<RangeType>* r) : ref(r)
  {
    it = ref->ranges.begin();
    if (it != ref->ranges.end()) range_it = it->begin();
  }
  bool operator==(const multi_range_iterator &mri) const {
    return mri.ref == ref && mri.it == it;
  }
  bool operator!=(const multi_range_iterator &i) const {
    return !(*this == i);
  }
  multi_range_iterator& operator+=(int steps) {
    if (ref != nullptr) {
      range_it += steps;
      if (ref->merged && range_it.clusterDone()) {
        nextCluster();
      }
    }
    return *this;
  }
  multi_range_iterator& operator++() { return (*this) += 1; }
  multi_range_iterator operator++(int) { auto self = *this; ++(*this); return self; }
  typename RangeType::type& operator*() { return *range_it; }
  bool done() const {
    return !ref || it == ref->ranges.end();
  }
  bool clusterDone() const {
    if (ref == nullptr)
      return true;
    if (ref->merged) {
      return done();
    }
    else
      return range_it.clusterDone();
  }
  void nextCluster() {
    if (!ref) return;
    range_it.nextCluster();
    if (range_it.done() && !done()) {
      it++;
      if(!done()) range_it = it->begin();
    }
  }
  multi_range<RangeType> cluster() {
    if (ref->merged)
      return *ref;
    else {
      return multi_range<RangeType>({ range_it.cluster() });
    }
  }
  multi_range<RangeType>* reference() {
    return ref;
  }

private:
  multi_range<RangeType>* ref;
  typename std::vector<RangeType>::iterator it;
  typename RangeType::iterator range_it;
};

//forward declaration
template<class T>
class range_set_iterator;

template<class T>
class range_set {
  range<T> m_limits;
  std::vector<std::vector<range<T> > > m_subsets;
  size_t m_cluster_size;

public:

  range_set(const std::vector<std::vector<range<T> > >& subsets = {}
    , const range<T>& limits = {0, -1, 1, 1})
    : m_subsets(subsets)
  {
    setLimits(limits);
    m_cluster_size = m_limits.clustersize;
  }

  inline void setMax(T max_val) {
    m_limits.max = max_val;
  }
  inline void setLimits(const range<T>& limits) {
    m_limits = limits;
  }
  inline void setSubsets(const std::vector<std::vector<range<T> > >& subsets) { m_subsets = subsets; }
  inline const range<T>& limits() const { return m_limits; }
  inline const std::vector<std::vector<range<T> > >& subsets() const { return m_subsets; }
  inline const T& clusterSize() const { return m_cluster_size; }
  inline const unsigned int& stepsize() const { return m_limits.stepsize; }
  inline range_set_iterator<T> begin() const { return range_set_iterator<T>(const_cast<range_set<T>* const>(this)); }
};

template<class T>
class range_set_iterator {

public:
  range_set_iterator(range_set<T>* const rs)
    : ref(rs), in_cluster(false), in_subset(false), reached_end(false)
    , subset_done(false), pos(rs->limits().min), only_subsets(rs->limits().min == -1), cluster_min(0), cluster_max(0)
    , id_len(3), last_range_identifier(""), last_subset_identifier("")
  {
    subset_index[0] = 0;
    subset_index[1] = 0;
    range_it = rs->limits().begin();
    const std::vector<std::vector<range<T> > >& subsets = ref->subsets();
    //if no bulk start at first subset
    if (only_subsets) {
      if (subsets.size() == 0) {
        subset_done = reached_end = true;
        return;
      }
      else {
        pos = subsets[0][0].min;
      }
    }
    start_pos = pos;
    //start subset
    for (size_t i = 0; i < subsets.size(); i++)
    {
      if (inSubset(i)) {
        in_subset = true;
        subset_index[0] = i;
        save_identifiers();
        return;
      }
    }
    //or cluster
    start_cluster();
    save_identifiers();
  }

  inline void setIdLen(int len) { id_len = len; }
  std::string rangeIdentifier(int idx2) {
    if (subsetDone())
      return last_range_identifier;
    T start, end;
    if (inSubset()) {
      start = ref->subsets()[subset_index[0]][idx2].min;
      end = ref->subsets()[subset_index[0]][idx2].max;
    }
    else {
      start = cluster_min;
      end = cluster_max;
    }

    start = ceil_pos(start);
    end = floor_pos(end);

    //No element within limits
    if (start == -1)
      return "";
    //Only one element within stepsize steps
    else if (end != -1 && start == end)
      return to_string(start, id_len);
    //Multiple active elements or unlimited range
    else
      return to_string(start, id_len) + ":" + (end == -1 ? "-1" : to_string(end, id_len))
        + (ref->stepsize() > 1 ? (":" + to_string(ref->stepsize(), 1)) : "");
  }
  inline std::string rangeIdentifier() { return rangeIdentifier(subset_index[1]); }

  std::string subsetIdentifier() {
    if (subsetDone())
      return last_subset_identifier;
    std::string identifier;
    if (inSubset()) {
      identifier = rangeIdentifier(0);
      for (int i = 1; i < currentSubset().size(); ++i) {
        std::string range_identifier = rangeIdentifier(i);
        if (!range_identifier.empty()) identifier += "+" + range_identifier;
      }
    }
    else {
      if (ref->clusterSize() == 1 || cluster_min + ref->stepsize() > cluster_max)
        identifier = to_string(pos, id_len);
      else {
        identifier = rangeIdentifier(id_len);
      }
    }
    return identifier;
  }
  inline std::vector<range<T> > currentSubset() {
    if (in_subset) return (ref->subsets())[subset_index[0]];
    else return { currentRange() };
  }
  inline range<T> currentRange() {
    if (in_subset) return currentSubset()[subset_index[1]];
    //otherwise in cluster
    else return { cluster_min, cluster_max };
  }
  inline bool done() { return reached_end; }
  inline bool subsetDone() { return subset_done; }
  inline void nextSubset() {
    if (!subset_done) {
      save_identifiers();
      if (in_subset && currentRange().max == -1) {
        subset_done = reached_end = true;
        return;
      }
      else
        while (!subsetDone()) ++(*this);
    }
    subset_done = false;
  }
  inline const T& operator*() { return pos; }
  inline bool inSubset(size_t i) {
    const std::vector<std::vector<range<T> > >& subsets = ref->subsets();
    for (int j = 0; j < subsets[i].size(); j++) {
      if (pos >= subsets[i][j].min && (subsets[i][j].max == -1 || pos <= subsets[i][j].max)) {
        return true;
      }
    }
    return false;
  }
  inline bool inSubset() {
    const std::vector<std::vector<range<T> > >& subsets = ref->subsets();
    for (size_t i = 0; i < subsets.size(); i++)
      if (inSubset(i)) return true;
    return false;
  }
  T operator++() {
    if (subset_done || reached_end) {
      return pos;
    }
    const T& stepsize = ref->stepsize();
    //range_it++;
    //pos = *range_it;
    pos += stepsize;
    if (ref->limits().max != -1 && pos > ref->limits().max) {
    //if(range_it.done()) {
      save_identifiers();
      subset_done = reached_end = true;
      pos = ref->limits().max;
      return pos;
    }
    if (in_subset)
    {
      if (pos <= currentRange().max || currentRange().max == -1)
        return pos;
      //if at end of current range in subset...
      else
      {
        //...jump to next range of subset...
        next_range();
        if (in_subset) return pos;
      }
    }
    //skip blocks from already processed subsets
    for (size_t i = 0; i < blocked_ranges.size(); ++i)
    {
      if (pos >= blocked_ranges[i].min && pos <= blocked_ranges[i].max) {
        save_identifiers();
        subset_done = true;
        pos = ceil_pos(blocked_ranges[i].max);
      }
    }
    //check if new range subset reached
    for (size_t i = 0; i < ref->subsets().size(); ++i)
    {
      if (inSubset(i)) {
        in_subset = true;
        in_cluster = false;
        subset_done = true;
        subset_index[0] = i;
        subset_index[1] = 0;
        return pos;
      }
    }
    //else continue or start new cluster
    if (pos > cluster_max) {
      save_identifiers();
      start_cluster();
      subset_done = true;
    }
    return pos;
  }
  inline T operator++(int) { return operator++(); }
  void setMax(T max_val) {
    const T& max_limit = ref->limits().max;
    if (max_limit == -1 || max_limit > max_val)
      ref->setMax(max_val);
    save_identifiers();
    if (in_subset && pos > max_val)
      finish_subset();
    if (pos > max_val)
      subset_done = reached_end = true;
  }

private:
  range_set<T>* const ref;
  typename range<T>::iterator range_it;
  std::vector<range<T> >blocked_ranges;
  size_t subset_index[2];
  bool in_subset;
  bool in_cluster;
  bool subset_done;
  bool reached_end;
  bool only_subsets;
  T cluster_min;
  T cluster_max;
  T pos;
  T start_pos;
  int id_len;
  std::string last_subset_identifier;
  std::string last_range_identifier;

  inline void start_cluster() {
    cluster_min = pos;
    if (ref->clusterSize() == 1)
      cluster_max = cluster_min;
    else {
      cluster_max = cluster_min + (ref->clusterSize() - 1) * ref->stepsize();
      if (!only_subsets && cluster_max > ref->limits().max) cluster_max = ref->limits().max;
    }
    in_cluster = true;
  }

  inline T next_range() {
    //TODO (Peter Janotta): if(!in_subset)...
    if (subset_index[1] < currentSubset().size() - 1) {
      subset_index[1]++;
      pos = ceil_pos(currentRange().min);
      if (pos <= currentRange().max || currentRange().max == -1)
        return pos;
      else
        return next_range();
    }
    else {
      finish_subset();
    }
  }

  inline void finish_subset() {
    //const std::vector<std::vector<range<T> > >& subsets = ref->subsets();
    for (size_t i = 1; i < currentSubset().size(); ++i)
      blocked_ranges.push_back(currentSubset()[i]);
    subset_index[1] = 0;
    if (subset_index[0] >= ref->subsets().size() - 1) {
      if (only_subsets) reached_end = true;
    }
    else {
      subset_index[0]++;
      if (only_subsets) {
        pos = ceil_pos(currentRange().min);
        if (currentRange().max != -1 && pos > currentRange().max) next_range();
      }
    }
    in_subset = false;
    subset_done = true;
  }

  inline T floor_pos(T pos) {
    T max_limit = ref->limits().max;
    if (pos == -1) {
      if (max_limit == -1) return -1;
      else return floor_pos(max_limit);
    }
    else if (max_limit != -1 && max_limit < pos)
      return floor_pos(max_limit);
    else
      return pos - ((pos - start_pos) % ref->stepsize());
  }

  inline T ceil_pos(T pos) {
    if (pos < 0) return 0;
    T max_limit = ref->limits().max;
    if (max_limit != -1 && pos > max_limit) return -1;
    return floor_pos(pos + ref->stepsize() - 1);
  }

  inline void save_identifiers() {
    last_range_identifier = rangeIdentifier();
    last_subset_identifier = subsetIdentifier();
  }
};

struct Color {
  float r, g, b;
  Color() : r(0), g(0), b(0) {};
  Color(float r, float g, float b) : r(r), g(g), b(b) {};
};

enum class ShowColormap : int {
  solid = 0,
  grey = 1,
  hsv = 2,
  jet = 3,
  hot = 4,
  rand = 5,
  shsv = 6,
  temp = 7
};

// The members initialized with {} can not have a sensible default in parse_args

struct color_settings {
  PointType ptype;
  int colorval = -1;
  bool explicit_coloring;
  ShowColormap colormap;
  range<float> colormap_values{ NAN, NAN };
  Color bgcolor;
  int scans_colored;
};

//struct dataset_settings;
//bool parse_dataset(std::string dss_def, dataset_settings &dss);

struct dataset_settings {
  enum data_type{
    SINGLE_SRC,
    MULTI_SRCS
  };

  data_type data_type;
  std::string data_source;
  IOType format;
  bool use_scanserver;
  range<int> scan_numbers;

  double scale;

  range<double> distance_filter;
  double octree_reduction_voxel;
  int octree_reduction_randomized_bucket{};
  int skip_files;
  int skip_points;
  int cluster_size;
  multi_range<multi_range<range<int> > > scan_ranges;

  color_settings coloring;

  // TODO make this an std::optional (C++17)
  int origin_type{};
  bool origin_type_set{};
  double sphere_radius;

  bool save_octree;
  bool load_octree;
  bool cache_octree;

  std::string objects_file_name{};
  std::string custom_filter{};
  std::string trajectory_file_name{};
  bool identity;

  dataset_settings *parent;
  std::vector<dataset_settings*> subsets;

  //init values
  dataset_settings()
    : data_source(""),
    format(UOS),
    use_scanserver(false),
    scan_numbers({ 0, -1 }),
    scale(0.01),
    distance_filter({ 0, -1 }),
    octree_reduction_voxel(0),
    octree_reduction_randomized_bucket(1),
    origin_type(0),
    origin_type_set(false),
    sphere_radius(0),
    save_octree(false),
    load_octree(false),
    cache_octree(false),
    objects_file_name(""),
    custom_filter(""),
    trajectory_file_name(""),
    identity(false),
    parent(nullptr)
  {}

  dataset_settings(dataset_settings *parent)
    : data_source(""), scan_numbers({ 0,-1 }), objects_file_name(""), trajectory_file_name(""), parent(parent)
  {
    if (parent != nullptr) {
      format = parent->format;
      use_scanserver = parent->use_scanserver;
      //scan_numbers = parent->scan_numbers;
      scale = parent->scale;
      distance_filter = parent->distance_filter;
      octree_reduction_voxel = parent->octree_reduction_voxel;
      octree_reduction_randomized_bucket = parent->octree_reduction_randomized_bucket;
      origin_type = parent->origin_type;
      origin_type_set = parent->origin_type_set;
      sphere_radius = parent->sphere_radius;
      save_octree = parent->save_octree;
      load_octree = parent->load_octree;
      cache_octree = parent->cache_octree;
      custom_filter = parent->custom_filter;
      identity = parent->identity;
      parent->subsets.push_back(this);
    }
  }

  void set(std::vector<std::string> subsets_def) {
    if (subsets_def.size() == 1) {
      set(subsets_def[0]);
      return;
    }
    else {
      data_type = MULTI_SRCS;
      for (auto it = subsets_def.begin(); it != subsets_def.end(); ++it) {
        dataset_settings *child_set = new dataset_settings(this);
        child_set->set(*it);
      }
    }
  }
  void set(std::string dss_def) {
    data_type = SINGLE_SRC;
    parse_dataset(dss_def, *this);
  }
  inline void clear() { subsets.clear(); }
  ~dataset_settings() { clear(); }
};

//#ifndef __RANGE_SET_PARSER_H__
//#include "parsers/range_set_parser.h"
//#endif

#endif //__DATASET_SETTINGS_H__
