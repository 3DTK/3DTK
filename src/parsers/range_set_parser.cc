#include "parsers/range_set_parser.h"
#include "slam6d/scan_settings.h"

#include <boost/phoenix/statement/if.hpp>
#include <boost/config/warning_disable.hpp>
#include <boost/spirit/include/qi.hpp>
#include <boost/spirit/include/phoenix_core.hpp>
#include <boost/spirit/include/phoenix_operator.hpp>
#include <boost/spirit/include/phoenix_object.hpp>
#include <boost/spirit/include/phoenix_stl.hpp>
#include <boost/spirit/include/phoenix_fusion.hpp>

#include <boost/fusion/tuple.hpp>

#include <iostream>

namespace qi = boost::spirit::qi;
namespace ascii = boost::spirit::ascii;
namespace phx = boost::phoenix;

template <typename T, typename Iterator>
struct range_parser : qi::grammar<Iterator, range<T>(), ascii::space_type>
{
  range_parser() : range_parser::base_type(range_def)
  {
    using qi::int_;
    using qi::lit;
    using qi::_1;
    using qi::_2;
    using qi::_val;
    using phx::construct;
    using phx::at_c;

    range_def = range_params[_val = construct<range<T> >(at_c<0>(_1), at_c<1>(_1), at_c<2>(_1), at_c<3>(_1))];
    range_params = qi::eps[_val = boost::fusion::make_tuple<T, T, size_t, size_t>(0, -1, 1, 1)]
      >> ( (int_[at_c<0>(_val) = _1] >> lit(':') >> (lit("-1")[at_c<1>(_val) = range<T>::UNLIMITED] | int_[at_c<1>(_val) = _1])) 
        >> -clustersize[phx::if_(_1 == -1)[at_c<2>(_val) = phx::val(range<T>::CLUSTER_ALL)].else_[at_c<2>(_val) = _1]]
        >> -stepsize[at_c<3>(_val) = _1]
      )
      | int_[at_c<0>(_val) = at_c<1>(_val) = _1];
    clustersize %= (*lit("-") >> lit("clustersize") >> -lit("=") >> int_) | ('[' >> int_ >> ']');
    stepsize %= ((*lit("-") >> lit("stepsize") >> -lit("=")) | ':') >> int_;
  }
  qi::rule<Iterator, range<T>(), ascii::space_type> range_def;
  qi::rule<Iterator, boost::fusion::tuple<T, T, size_t, size_t>(), ascii::space_type> range_params;
  qi::rule<Iterator, size_t(), ascii::space_type> clustersize;
  qi::rule<Iterator, size_t(), ascii::space_type> stepsize;
};

template struct range_parser<int, std::string::const_iterator>;

template<typename T> bool parse_range(std::string range_def, range<T> &r)
{
  if (range_def.empty()) return true;

  using boost::spirit::ascii::space;
  typedef std::string::const_iterator iterator_type;
  typedef range_parser<T, iterator_type> range_parser;

  range_parser g;
  std::string::const_iterator iter = range_def.begin();
  std::string::const_iterator end = range_def.end();
  bool success = qi::phrase_parse(iter, end, g, space, r);
  if (!success || iter != end)
    std::cerr << "Error parsing range from string \"" << range_def << "\"" << std::endl;
  return success;
}

template bool parse_range<int>(std::string range_def, range<int> &r);

template <typename RangeType, typename Iterator>
struct multi_range_parser : qi::grammar<Iterator, multi_range<RangeType>(), ascii::space_type>
{
  multi_range_parser() : multi_range_parser::base_type(_multi_range)
  {
    _multi_range = (range_def % '+')[qi::_val = phx::construct<multi_range<RangeType> >(qi::_1, phx::val(false))]
      | (qi::lit('[') >> (range_def % '+') >> qi::lit(']'))[qi::_val = phx::construct<multi_range<RangeType> >(qi::_1, phx::val(true))]
      | (qi::lit("[]") | qi::lit("merged"))[qi::_val = phx::construct<multi_range<RangeType> >(phx::val<std::vector<RangeType> >({ RangeType() }), phx::val(true))];
  }

  typename RangeType::parser range_def;
  qi::rule<typename RangeType::parser::iterator_type, multi_range<RangeType>(), ascii::space_type> _multi_range;
};

template struct multi_range_parser<range<int>, std::string::const_iterator>;
template struct multi_range_parser<multi_range<range<int> >, std::string::const_iterator>;

template<typename RangeType> bool parse_multi_range(std::string mr_def, multi_range<RangeType> &mr)
{
  if (mr_def.empty()) return true;

  using boost::spirit::ascii::space;
  std::string::const_iterator iter = mr_def.begin();
  std::string::const_iterator end = mr_def.end();
  multi_range_parser<RangeType, typename RangeType::parser::iterator_type> mr_parser;

  bool r = qi::phrase_parse(iter, end
    , mr_parser
    , space, mr);
  if (!r || iter != end)
    std::cerr << "Error parsing range_set from string \"" << mr_def << "\"" << std::endl;
  //else {
  //  std::cout << "-------------------------\n";
  //  std::cout << "Parsing succeeded\n";
  //  std::cout << "GOT #ranges: " << mr.ranges.size() << ", merged: " << (mr.merged ? "true" : "false") << std::endl;
  //  std::cout << "-------------------------\n";
  //}
  return r;
}

template bool parse_multi_range<range<int> >(std::string mr_def, multi_range<range<int> > &mr);
template bool parse_multi_range<multi_range<range<int> > >(std::string mr_def, multi_range<multi_range<range<int> > > &mr);

/*
template <typename T, typename Iterator>
struct range_set_parser : qi::grammar<Iterator, range_set<T>(), ascii::space_type>
{
  typedef std::vector<range<T> > merged_subset;
  typedef std::vector<merged_subset> merged_subsets;
  typedef boost::fusion::tuple<merged_subsets, range<T>> range_set_params;

  range_set_parser() : range_set_parser::base_type(range_set_def)
  {
    using qi::int_;
    using qi::lit;
    using qi::_1;
    using qi::_2;
    using qi::_val;

    using phx::at_c;
    using phx::push_back;
    using phx::construct;
    using phx::ref;
    using phx::val;

    const range<T> limit({ 0,-1 });
    //const T std_stepsize = 1;
    //const T std_clustersize = 1;

    range_set_def = (range_set_with_limits | range_set_merged_only)[_val = construct<range_set<T> >(at_c<0>(_1), at_c<1>(_1))]; // , at_c<2>(_1))]; // , at_c<3>(_1))];

    range_set_with_limits =
      qi::eps[_val = val(boost::fusion::make_tuple(merged_subsets(), range<T>()))]
      >> range_def[at_c<1>(_val) = _1]
      >> *(merged_subset_def[push_back(at_c<0>(_val), _1)]); // | clustersize[at_c<2>(_val) = _1]); // | stepsize[at_c<3>(_val) = _1]);

    range_set_merged_only =
      // set limits {-1, -1}
      qi::eps[_val = val(boost::fusion::make_tuple(merged_subsets(), range<T>(-1, -1), 1))]
      >> +(merged_subset_def[push_back(at_c<0>(_val), _1)]); // | clustersize[at_c<2>(_val) = _1]); // | stepsize[at_c<3>(_val) = _1]);

    merged_subset_def %= lit('[') >> (range_def % '+')[_val = _1] >> lit(']') || (lit("merged") | lit("[]"))[push_back(_val, val(range<T>(0, -1)))];
    //range_def = (int_ >> lit(':') >> int_)[_val = construct<range<T> >(_1, _2)];

    //stepsize %= ((*lit("-") >> lit("stepsize") >> -lit("=")) | ':') >> int_;
    //clustersize %= (*lit("-") >> lit("clustersize") >> -lit("=") >> int_) | ('[' >> int_ >> ']');
  }

  qi::rule<Iterator, range_set<T>(), ascii::space_type> range_set_def;
  qi::rule<Iterator, range_set_params(), ascii::space_type> range_set_with_limits;
  qi::rule<Iterator, range_set_params(), ascii::space_type> range_set_merged_only;
  qi::rule<Iterator, merged_subset(), ascii::space_type> merged_subset_def;
  range_parser<int, Iterator> range_def;
  qi::rule<Iterator, T(), ascii::space_type> stepsize;
  qi::rule<Iterator, T(), ascii::space_type> clustersize;
};

template struct range_set_parser<int, std::string::const_iterator>;

template<typename T> bool parse_range_set(std::string rs_def, range_set<T> &rs)
{
  if (rs_def.empty()) return true;

  using boost::spirit::ascii::space;
  using boost::fusion::tuple;
  using boost::fusion::get;
  typedef std::string::const_iterator iterator_type;
  typedef range_set_parser<T, iterator_type> range_set_parser;
  typedef std::vector<range<T> > merged_subset;
  typedef std::vector<merged_subset> merged_subsets;
  typedef tuple<merged_subsets, range<T>, T, T> range_set_params;

  range_set_params params;
  range_set_parser g;
  std::string::const_iterator iter = rs_def.begin();
  std::string::const_iterator end = rs_def.end();
  bool r = qi::phrase_parse(iter, end, g, space, rs);
  if (!r || iter != end)
    std::cerr << "Error parsing range_set from string \"" << rs_def << "\"" << std::endl;
  //else {
  //  std::cout << "-------------------------\n";
  //  std::cout << "Parsing succeeded\n";
  //  std::cout << "GOT #subsets: " << rs.subsets().size() << ", clustersize: " << rs.clusterSize() << ", stepsize: " << rs.stepsize() << std::endl;
  //  std::cout << "-------------------------\n";
  //}
  return r;
}

template bool parse_range_set<int>(std::string rs_def, range_set<int> &rs);
*/

template <typename Iterator>
struct dataset_parser : qi::grammar<Iterator, boost::fusion::tuple<std::string, multi_range_set>(), ascii::space_type>
{
  typedef boost::fusion::tuple<std::string, multi_range_set> dataset_params;

  dataset_parser() : dataset_parser::base_type(dataset_def)
  {
    using qi::_1;
    using qi::_2;
    using qi::_val;
    using qi::lit;
    using qi::lexeme;
    using qi::char_;

    using phx::at_c;
    using phx::push_back;
    using phx::construct;
    using phx::ref;
    using phx::val;

    dataset_def =
      qi::eps[_val = boost::fusion::make_tuple<std::string, multi_range_set>("", multi_range_set())]
      >> source_def[at_c<0>(_val) = _1] >> -(lit('[') >> range_set_def[at_c<1>(_val) = _1] >> ']');

    source_def = lexeme[+(char_ - lit('['))];
  }

  qi::rule<Iterator, dataset_params(), ascii::space_type> dataset_def;
  qi::rule<Iterator, std::string(), ascii::space_type> source_def;
  multi_range_parser<multi_range<range<int> >, Iterator> range_set_def;
  //multi_range_parser<multi_range_set, Iterator> range_set_def;
};

template struct dataset_parser<std::string::const_iterator>;

bool parse_dataset(std::string dss_def, dataset_settings &dss)
{
  if (dss_def.empty()) return false;

  using boost::spirit::ascii::space;
  using boost::fusion::get;
  typedef std::string::const_iterator iterator_type;
  typedef dataset_parser<iterator_type> dataset_parser;

  boost::fusion::tuple<std::string, multi_range_set> params;
  dataset_parser g;

  std::string::const_iterator iter = dss_def.begin();
  std::string::const_iterator end = dss_def.end();
  bool r = phrase_parse(iter, end, g, space, params);
  if (!r || iter != end)
    std::cerr << "Error parsing dataset settings from string \"" << dss_def << "\"" << std::endl;
  else {
    dss.data_source = get<0>(params);
    dss.scan_ranges = get<1>(params);
  }

  return r;
}
