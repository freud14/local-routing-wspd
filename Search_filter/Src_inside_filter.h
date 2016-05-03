#ifndef SRC_INSIDE_FILTER
#define SRC_INSIDE_FILTER
#include <map>
#include <CGAL/WSPD.h>
#include "Point_wsp.h"
#include "Base_search_filter.h"

template <typename Traits>
class Src_inside_filter : public Base_search_filter<Traits>
{
public:
  typedef Point_wsp<Traits>                                       Point_wsp_type;

  typedef typename CGAL::WSPD<Traits>                             WSPD;
  typedef typename WSPD::Well_separated_pair                      Well_separated_pair;
  typedef typename WSPD::Node_const_handle                        Node_const_handle;

  typedef WSP_iterator<Traits>                                    WSP_iterator_type;
  typedef typename std::vector<Node_const_handle>::const_iterator Node_const_iterator;
  typedef typename std::vector<Point_wsp_type*>                   Point_wsp_vector;
  typedef typename Point_wsp_vector::const_iterator               Point_wsp_const_iterator;
public:
  Src_inside_filter(Point_wsp_type* src_, Point_wsp_type* dest_, std::map<Node_const_handle, std::vector<Point_wsp_type*> >& node_representatives_)
        : Base_search_filter<Traits>(src_, dest_), node_representatives(node_representatives_) { }

  virtual std::vector<Point_wsp_type*> filter(Point_wsp_type* point, const std::vector<Point_wsp_type*>& neighbors) {
    std::vector<Point_wsp_type*> candidates;
    for(WSP_iterator_type pairIt = point->representative_of_begin(); pairIt != point->representative_of_end(); pairIt++) {
      Node_const_handle from = pairIt.from();
      Node_const_handle to = pairIt.to();
      if(this->src->is_inside(from->bounding_box())) {
        candidates.insert(candidates.end(), node_representatives[to].begin(), node_representatives[to].end());
      }
    }

    std::sort(candidates.begin(), candidates.end());
    std::vector<Point_wsp_type*> n(neighbors);
    std::sort(n.begin(), n.end());

    std::vector<Point_wsp_type*> ret;
    std::set_intersection(candidates.begin(), candidates.end(), n.begin(), n.end(), std::back_inserter(ret));
    return ret;
  }

  virtual std::string to_string() const { return "Src_inside_filter"; }
private:
  std::map<Node_const_handle, std::vector<Point_wsp_type*> >& node_representatives;
};

#endif // SRC_INSIDE_FILTER
