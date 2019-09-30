#ifndef __RANGE_SET_PARSER_H__
#define __RANGE_SET_PARSER_H__

#include <string>

//forward definitions
template<typename T> struct range;
template<typename T> class range_set;
template<typename T> struct multi_range;
struct dataset_settings;

template <typename T, typename Iterator>
struct range_parser;

template<typename T> bool parse_range(std::string range_def, range<T> &r);

template <typename T, typename Iterator>
struct multi_range_parser;

template<typename RangeType> bool parse_multi_range(std::string mr_def, multi_range<RangeType> &mr);

/*
template <typename T, typename Iterator>
struct range_set_parser;

template<typename T> bool parse_range_set(std::string rs_def, range_set<T> &rs);
*/

template <typename Iterator>
struct dataset_parser;

bool parse_dataset(std::string dss_def, dataset_settings &dss);

#endif //__RANGE_SET_PARSER_H__
