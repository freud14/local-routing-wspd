#ifndef SRC_INSIDE_FILTER
#define SRC_INSIDE_FILTER
#include <CGAL/WSPD.h>
#include "Point_wsp.h"
#include "Base_search_filter.h"

template <typename Traits>
class Src_inside_filter : public Base_search_filter<Traits>
{
public:
  typedef Point_wsp<Traits>                                       Point_wsp_type;

  typedef typename std::vector<Point_wsp_type*>                   Point_wsp_vector;
  typedef typename Point_wsp_vector::const_iterator               Point_wsp_const_iterator;
public:
  Src_inside_filter(Point_wsp_type* src_, Point_wsp_type* dest_) : Base_search_filter<Traits>(src_, dest_) { }

  virtual std::vector<Point_wsp_type*> filter(Point_wsp_type* point, const std::vector<Point_wsp_type*>& neighbors) {
    return neighbors;
  }

  virtual std::string to_string() const { return "Src_inside_filter"; }
};

#endif // SRC_INSIDE_FILTER
