#ifndef WSP_ITERATOR_H
#define WSP_ITERATOR_H
#include <iterator>

template <typename Traits>
class Point_wsp;

template <typename Traits>
class WSP_iterator : public std::iterator<std::input_iterator_tag, typename CGAL::WSPD<Traits>::Well_separated_pair> {
private:
  typedef CGAL::WSPD<Traits>                                      WSPD;
  typedef typename WSPD::Node_const_handle                        Node_const_handle;
  typedef typename WSPD::Well_separated_pair                      Well_separated_pair;

  typedef Point_wsp<Traits>                                       Point_wsp_type;
  typedef typename std::vector<Well_separated_pair>::const_iterator     iterator;

  friend Point_wsp_type;

  WSP_iterator(const Point_wsp_type& point, iterator it_) :p(point), it(it_) { }
public:
  WSP_iterator(const WSP_iterator& rhs) :p(rhs.p), it(rhs.it) { }

  inline WSP_iterator& operator++() { ++it; return *this; }
  inline WSP_iterator operator++(int) { WSP_iterator ret = *this; ++it; return ret; }
  inline bool operator==(WSP_iterator rhs) const { return it == rhs.it; }
  inline bool operator!=(WSP_iterator rhs) const { return it != rhs.it; }
  Node_const_handle from() const {
    if(p.is_inside(it->a()->bounding_box())) {
      return it->a();
    }
    else {
      return it->b();
    }
  }
  Node_const_handle to() const {
    if(p.is_inside(it->a()->bounding_box())) {
      return it->b();
    }
    else {
      return it->a();
    }
  }
  const Well_separated_pair& operator*() const {return *it;}
  const Well_separated_pair& operator->() const {return *it;}
private:
  const Point_wsp_type& p;
  iterator it;
};

#endif // WSP_ITERATOR_H
