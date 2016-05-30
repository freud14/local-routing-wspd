#ifndef BOUNDED_BIGGEST_BBOX_FILTER
#define BOUNDED_BIGGEST_BBOX_FILTER
#include <CGAL/WSPD.h>
#include "Point_wsp.h"
#include "Base_search_filter.h"

template <typename Traits>
class Bounded_biggest_bbox_filter : public Base_search_filter<Traits>
{
public:
  typedef typename Traits::K                                      K;
  typedef typename K::FT                                          FT;
  typedef typename K::Point_2                                     Point_2;
  typedef typename K::Iso_rectangle_2                             Iso_rectangle_2;

  typedef Point_wsp<Traits>                                       Point_wsp_type;

  typedef typename CGAL::WSPD<Traits>                             WSPD;
  typedef typename WSPD::Well_separated_pair                      Well_separated_pair;
  typedef typename WSPD::Node_const_handle                        Node_const_handle;

  typedef WSP_iterator<Traits>                                    WSP_iterator_type;
  typedef typename std::vector<Node_const_handle>::const_iterator Node_const_iterator;
  typedef typename std::vector<Point_wsp_type*>                   Point_wsp_vector;
  typedef typename std::vector<Point_wsp_type*>::const_iterator   Point_wsp_const_iterator;
public:
  Bounded_biggest_bbox_filter(Point_wsp_type* src_, Point_wsp_type* dest_, FT separation_ratio) : Base_search_filter<Traits>(src_, dest_), s(separation_ratio) { }

  virtual std::vector<Point_wsp_type*> filter(Point_wsp_type* point, const std::vector<Point_wsp_type*>& neighbors) {
    std::vector<Point_wsp_type*> ret;

    Iso_rectangle_2 biggest_box = point->rep_biggest_box();
    Point_wsp_type* cur_point = NULL;
    Iso_rectangle_2 cur_box(Point_2(0,0), Point_2(0,0));

    for(Point_wsp_const_iterator it = neighbors.begin(); it != neighbors.end(); it++) {
      Point_wsp_type* cur_new_point = cur_point;
      Iso_rectangle_2 cur_new_box = cur_box;


      Point_wsp_type* new_point = *it;
      bool is_candidate = false;
      for(WSP_iterator_type pairIt = new_point->representative_of_begin(); pairIt != new_point->representative_of_end(); pairIt++) {
        Node_const_handle from = pairIt.from();
        Node_const_handle to = pairIt.to();
        Iso_rectangle_2 new_box = from->bounding_box();
        if(this->src->is_inside(new_box) &&
              biggest_box.area() < new_box.area() &&
              (cur_new_point == NULL || new_box.area() > cur_new_box.area() || new_box.area() == cur_new_box.area()) &&
              satisfy_bound(from)) {
          if(cur_new_point != NULL && new_box.area() > cur_new_box.area()) {
            ret.clear();
          }
          cur_new_point = new_point;
          cur_new_box = new_box;
          is_candidate = true;
        }
      }

      if(is_candidate) {
        cur_point = cur_new_point;
        cur_box = cur_new_box;
        ret.push_back(new_point);
      }
    }
    return ret;
  }

  virtual std::string to_string() const { return "Bounded_biggest_bbox_filter"; }

private:
  bool satisfy_bound(Node_const_handle node) const {
    FT distance = CGAL::squared_distance(node->center(), this->dest->point());
    return distance >= (s + 1) * (s + 1) * node->squared_radius();
  }

private:
  FT s;
};

#endif // BOUNDED_BIGGEST_BBOX_FILTER
