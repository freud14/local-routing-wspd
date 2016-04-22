#ifndef BASE_SEARCH_FILTER_H
#define BASE_SEARCH_FILTER_H
#include <iostream>
#include <vector>
#include "Point_wsp.h"

template <typename Traits>
class Base_search_filter
{
public:
  typedef Point_wsp<Traits> Point_wsp_type;
public:
  Base_search_filter(Point_wsp_type* src_, Point_wsp_type* dest_) : src(src_), dest(dest_) { }
  virtual std::vector<Point_wsp_type*> filter(Point_wsp_type* point, const std::vector<Point_wsp_type*>& neighbors) = 0;
  virtual std::string to_string() const { return "Base_search_filter"; }
  virtual ~Base_search_filter() { }
protected:
  Point_wsp_type* src;
  Point_wsp_type* dest;
};

template <typename Traits>
std::ostream& operator<<(std::ostream& os, const Base_search_filter<Traits>& filter) {
    os << filter.to_string();
    return os;
}

template <typename Traits>
std::ostream& operator<<(std::ostream& os, const Base_search_filter<Traits>* filter) {
    os << *filter;
    return os;
}

#endif // BASE_SEARCH_FILTER_H
