#ifndef WSP_ITERATOR_H
#define WSP_ITERATOR_H
#include "Point_wsp.h"

template <typename Traits>
class WSP_iterator {
private:
  typedef typename CGAL::WSPD<Traits>                             WSPD;
  typedef typename WSPD::Node                                     Node;
  typedef typename WSPD::Well_separated_pair                      Well_separated_pair;

  typedef Point_wsp<Traits>                                       Point_wsp_type;
  typedef typename std::vector<Well_separated_pair>::const_iterator     iterator;

  WSP_iterator(const Point_wsp_type& point, iterator it_) :p(point), it(it_) { }
public:
  inline static WSP_iterator begin(const Point_wsp_type& point) {
    return WSP_iterator(point, point.representative_of().begin());
  }
  inline static WSP_iterator end(const Point_wsp_type& point) {
    return WSP_iterator(point, point.representative_of().end());
  }
  inline WSP_iterator& operator++() { ++it; return this; }
  inline WSP_iterator operator++(int) { WSP_iterator ret = *this; ++it; return ret; }
  inline bool operator==(WSP_iterator rhs) { return it == rhs.it; }
  inline bool operator!=(WSP_iterator rhs) { return it != rhs.it; }
  const Node* from() {
    if(it->a()->bounding_box().bounded_side(p) != -1) {
      return it->a();
    }
    else {
      return it->b();
    }
  }
  const Node* to() {
    if(it->a()->bounding_box().bounded_side(p) != -1) {
      return it->b();
    }
    else {
      return it->a();
    }
  }
private:
  const Point_wsp_type& p;
  iterator it;
};

#endif // WSP_ITERATOR_H
