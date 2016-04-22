#ifndef EUCLIDEAN_FILTER
#define EUCLIDEAN_FILTER
#include <CGAL/WSPD.h>
#include "Point_wsp.h"
#include "Base_search_filter.h"

template <typename Traits>
class Euclidean_filter : public Base_search_filter<Traits>
{
public:
  typedef Point_wsp<Traits>                                       Point_wsp_type;

  typedef typename std::vector<Point_wsp_type*>                   Point_wsp_vector;
  typedef typename Point_wsp_vector::const_iterator               Point_wsp_const_iterator;
public:
  Euclidean_filter(Point_wsp_type* src_, Point_wsp_type* dest_) : Base_search_filter<Traits>(src_, dest_) { }

  virtual std::vector<Point_wsp_type*> filter(Point_wsp_type* point, const std::vector<Point_wsp_type*>& neighbors) {
    std::vector<Point_wsp_type*> ret;
    Point_wsp_type* cur_point = NULL;

    for(Point_wsp_const_iterator it = neighbors.begin(); it != neighbors.end(); it++) {
      Point_wsp_type* new_point = *it;

      if(cur_point == NULL || CGAL::squared_distance(new_point->point(), this->dest->point()) <= CGAL::squared_distance(cur_point->point(), this->dest->point())) {
        if(cur_point == NULL || CGAL::squared_distance(new_point->point(), this->dest->point()) < CGAL::squared_distance(cur_point->point(), this->dest->point())) {
          ret.clear();
          cur_point = new_point;
        }
        ret.push_back(new_point);
      }
    }
    return ret;
  }

  virtual std::string to_string() const { return "Euclidean_filter"; }
};

#endif // EUCLIDEAN_FILTER
