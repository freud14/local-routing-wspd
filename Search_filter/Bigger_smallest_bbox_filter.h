#ifndef BIGGER_SMALLEST_BBOX_FILTER
#define BIGGER_SMALLEST_BBOX_FILTER
#include <CGAL/enum.h>
#include <CGAL/WSPD.h>
#include "Point_wsp.h"
#include "Base_search_filter.h"

template <typename Traits>
class Bigger_smallest_bbox_filter : public Base_search_filter<Traits>
{
public:
  typedef typename Traits::K                                      K;
  typedef typename K::Point_2                                     Point_2;
  typedef typename K::Iso_rectangle_2                             Iso_rectangle_2;

  typedef Point_wsp<Traits>                                       Point_wsp_type;

  typedef typename CGAL::WSPD<Traits>                             WSPD;
  typedef typename WSPD::Node_const_handle                        Node_const_handle;

  typedef WSP_iterator<Traits>                                    WSP_iterator_type;
  typedef typename std::vector<Node_const_handle>::const_iterator Node_const_iterator;
  typedef typename std::vector<Point_wsp_type*>                   Point_wsp_vector;
  typedef typename std::vector<Point_wsp_type*>::const_iterator   Point_wsp_const_iterator;
public:
  Bigger_smallest_bbox_filter(Point_wsp_type* src_, Point_wsp_type* dest_) : Base_search_filter<Traits>(src_, dest_) { }

  virtual std::vector<Point_wsp_type*> filter(Point_wsp_type* point, const std::vector<Point_wsp_type*>& neighbors) {
    Iso_rectangle_2 biggest_box = point->rep_biggest_box();

    Point_wsp_type* cur_point = NULL;
    Iso_rectangle_2 cur_box(Point_2(0,0), Point_2(0,0));

    std::vector<Point_wsp_type*> ret;
    for(Point_wsp_const_iterator it = neighbors.begin(); it != neighbors.end(); it++) {
      Point_wsp_type* new_point = *it;
      bool is_candidate = false;
      for(WSP_iterator_type pairIt = new_point->representative_of_begin(); pairIt != new_point->representative_of_end(); pairIt++) {
        Node_const_handle nodeFrom = pairIt.from();
        Iso_rectangle_2 new_box = nodeFrom->bounding_box();
        if(this->src->is_inside(new_box) &&
              biggest_box.area() < new_box.area() &&
              (cur_point == NULL || new_box.area() < cur_box.area() || new_box.area() == cur_box.area())) {
          if(cur_point != NULL && new_box.area() < cur_box.area()) {
            ret.clear();
          }
          cur_point = new_point;
          cur_box = new_box;
          is_candidate = true;

        }
      }

      if(is_candidate) {
        ret.push_back(new_point);
      }
    }
    return ret;
  }

  virtual std::string to_string() const { return "Bigger_smallest_bbox_filter"; }
};

#endif // BIGGER_SMALLEST_BBOX_FILTER
