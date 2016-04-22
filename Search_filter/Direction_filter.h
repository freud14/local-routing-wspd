#ifndef DIRECTION_FILTER
#define DIRECTION_FILTER
#include <CGAL/WSPD.h>
#include "Point_wsp.h"
#include "Base_search_filter.h"

template <typename Traits>
class Direction_filter : public Base_search_filter<Traits>
{
public:
  typedef typename Traits::K                                      K;
  typedef typename K::Direction_2                                 Direction_2;

  typedef Point_wsp<Traits>                                       Point_wsp_type;

  typedef typename std::vector<Point_wsp_type*>                   Point_wsp_vector;
  typedef typename Point_wsp_vector::const_iterator               Point_wsp_const_iterator;
public:
  Direction_filter(Point_wsp_type* src_, Point_wsp_type* dest_) : Base_search_filter<Traits>(src_, dest_) { }

  virtual std::vector<Point_wsp_type*> filter(Point_wsp_type* point, const std::vector<Point_wsp_type*>& neighbors) {
    if(neighbors.size() == 0) return neighbors;

    Direction_2 gen_direction(this->dest->point() - point->point());
    Direction_2 cur_direction(neighbors[0]->point() - point->point());

    std::vector<Point_wsp_type*> ret;
    for(Point_wsp_const_iterator it = neighbors.begin(); it != neighbors.end(); it++) {
      Point_wsp_type* new_point = *it;

      Direction_2 new_direction(new_point->point() - point->point());
      if(!new_direction.counterclockwise_in_between(gen_direction, -gen_direction)) {
        new_direction = Direction_2(2 * ((new_direction.vector() * gen_direction.vector()) / gen_direction.vector().squared_length()) * gen_direction.vector() - new_direction.vector());
      }

      if(new_direction == cur_direction || new_direction.counterclockwise_in_between(gen_direction, cur_direction)) {
        if(new_direction != cur_direction) {
          cur_direction = new_direction;
          ret.clear();
        }
        ret.push_back(new_point);
      }
    }
    return ret;
  }

  virtual std::string to_string() const { return "Direction_filter"; }
};

#endif // DIRECTION_FILTER
