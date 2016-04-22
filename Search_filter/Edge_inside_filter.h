#ifndef EDGE_INSIDE_FILTER
#define EDGE_INSIDE_FILTER
#include <CGAL/WSPD.h>
#include "Point_wsp.h"
#include "Base_search_filter.h"

template <typename Traits>
class Edge_inside_filter : public Base_search_filter<Traits>
{
public:
  typedef typename Traits::K                                      K;
  typedef typename K::Iso_rectangle_2                             Iso_rectangle_2;
  typedef Point_wsp<Traits>                                       Point_wsp_type;

  typedef typename std::vector<Point_wsp_type*>                   Point_wsp_vector;
  typedef typename Point_wsp_vector::const_iterator               Point_wsp_const_iterator;
public:
  Edge_inside_filter(Point_wsp_type* src_, Point_wsp_type* dest_) : Base_search_filter<Traits>(src_, dest_) { }

  virtual std::vector<Point_wsp_type*> filter(Point_wsp_type* point, const std::vector<Point_wsp_type*>& neighbors) {
    std::vector<Point_wsp_type*> ret;
    Iso_rectangle_2 biggest_box = point->rep_biggest_box();

    for(Point_wsp_const_iterator it = neighbors.begin(); it != neighbors.end(); it++) {
      Point_wsp_type* new_point = *it;

      if(biggest_box.bounded_side(*new_point) != -1) {
        ret.push_back(new_point);
      }
    }

    if(ret.empty()) {
      return neighbors;
    }
    else {
      return ret;
    }
  }

  virtual std::string to_string() const { return "Edge_inside_filter"; }
};

#endif // EDGE_INSIDE_FILTER
