#ifndef POINT_WSP_H
#define POINT_WSP_H
#include <CGAL/WSPD.h>
#include <CGAL/enum.h>

template <typename Traits>
class Point_wsp
{
public:
  typedef CGAL::WSPD<Traits>                                      WSPD;
  typedef typename WSPD::Well_separated_pair                      Well_separated_pair;
  typedef typename WSPD::Node                                     Node;

  typedef typename Traits::K K;
  typedef typename K::FT FT;
  typedef typename K::Point_2 Point_2;
  typedef typename K::Iso_rectangle_2 Iso_rectangle_2;

public:
  Point_wsp() : point_(0,0), number_(0), rep_biggest_box_(Point_2(0, 0), Point_2(0,0)) { }

  Point_wsp(Point_2 p, int n) : point_(p), number_(n), rep_biggest_box_(Point_2(0, 0), Point_2(0,0)) { }

  void point(Point_2 p) {
    point_ = p;
  }

  Point_2 point() const {
    return point_;
  }

  void number(int n) {
    number_ = n;
  }

  int number() const {
    return number_;
  }

  void add_pair(Well_separated_pair pair, const Node* from, const Node* to) {
    pairs_.push_back(pair);
    if(is_representative(from)) {
      representative_of_.push_back(pair);
      rep_froms_.push_back(from);
      rep_tos_.push_back(to);

      Iso_rectangle_2 bbox = from->bounding_box();
      if(rep_biggest_box_.area() <= bbox.area()) {
        rep_biggest_box_ = bbox;
      }
    }
  }

  bool is_representative(const Node* node) const {
    return node->bounding_box().has_on_boundary(point_);
  }

  const std::vector<Well_separated_pair>& pairs() const {
    return pairs_;
  }

  const std::vector<Well_separated_pair>& representative_of() const {
    return representative_of_;
  }

  const std::vector<const Node*>& rep_froms() const {
    return rep_froms_;
  }

  const std::vector<const Node*>& rep_tos() const {
    return rep_tos_;
  }

  Iso_rectangle_2 rep_biggest_box() const {
    return rep_biggest_box_;
  }

  bool operator==(const Point_wsp& p) const {
    return p.point_ == point_;
  }

  bool operator!=(const Point_wsp& p) const {
    return !operator==(p);
  }

  operator Point_2() const {
    return point_;
  }

  operator int() const {
    return number_;
  }
private:
  Point_2 point_;
  int number_;
  std::vector<Well_separated_pair> pairs_;
  std::vector<Well_separated_pair> representative_of_;
  std::vector<const Node*> rep_froms_;
  std::vector<const Node*> rep_tos_;
  Iso_rectangle_2 rep_biggest_box_;
};

#endif // POINT_WSP_H
