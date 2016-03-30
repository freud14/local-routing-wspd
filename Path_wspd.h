#ifndef PATH_WSPD_H
#define PATH_WSPD_H
#include <iostream>
#include "util.h"
#include <CGAL/Split_tree.h>
#include <CGAL/WSPD.h>
#include <map>
#include "Point_wsp.h"

template <typename Traits>
class Path_wspd : public CGAL::WSPD<Traits>
{
private:
  typedef typename Traits::K                                      K;
  typedef typename K::Point_2                                     Point_2;
  typedef typename K::Circle_2                                    Circle_2;
  typedef typename K::Iso_rectangle_2                             Iso_rectangle_2;

  typedef typename CGAL::WSPD<Traits>                             WSPD;
public:
  typedef typename WSPD::FT                                       FT;
  typedef typename WSPD::Split_tree::Point_container              Point_container;
  typedef typename WSPD::Node                                     Node;
  typedef typename WSPD::Well_separated_pair                      Well_separated_pair;
  typedef typename WSPD::Well_separated_pair_iterator             Well_separated_pair_iterator;
  typedef typename WSPD::Point_vector_iterator                    Point_vector_iterator;

private:
  typedef Point_wsp<Traits>                                       Point_wsp_type;
  typedef typename std::vector<const Node*>::const_iterator       Node_const_iterator;
  typedef typename std::vector<Point_wsp_type*>::const_iterator   Point_wsp_const_iterator;
public:
  Path_wspd(int d, FT separation_ratio) : WSPD(d, separation_ratio) { }

  template <class InputIterator>
  Path_wspd(int d, FT separation_ratio, InputIterator begin, InputIterator end) : WSPD(d, separation_ratio, begin, end) { }

  virtual void compute() const {
    if(!this->computed) {
      WSPD::compute();

      point_indices.clear();
      points_to_points_wsp.clear();
      node_representatives.clear();
      points_wsp.clear();
      for(int i = 0; i < this->points.size(); i++) {
        Point_2 p = this->points[i];
        point_indices[p] = i;
        points_wsp.push_back(Point_wsp_type(p, i));
      }
      for(int i = 0; i < this->points.size(); i++) {
        Point_2 p = this->points[i];
        points_to_points_wsp[p] = &points_wsp[i];
      }
      for(Well_separated_pair_iterator it = this->wspd_begin(); it != this->wspd_end(); it++) {
        Well_separated_pair& pair = *it;
        const Node* node1 = pair.first;
        const Node* node2 = pair.second;
        compute_points_wsp(pair, node1, node2);
        compute_points_wsp(pair, node2, node1);
      }
    }
  }

  void compute_points_wsp(Well_separated_pair& pair, const Node* from, const Node* to) const {
    for(typename Point_container::const_iterator it = from->point_container().begin(); it != from->point_container().end(); it++) {
      Point_2 p = **it;
      points_to_points_wsp[p]->add_pair(pair, from, to);
      if(points_to_points_wsp[p]->is_representative(from)) {
        node_representatives[from].push_back(points_to_points_wsp[p]);
      }
    }
  }

  std::vector<int> find_path(int srcIndex, int destIndex) const {
    std::ostream cnull(NULL);
    return find_path(srcIndex, destIndex, cnull, false);
  }

  std::vector<int> find_path(int srcIndex, int destIndex, std::ostream& out, bool debug) const {
    compute();
    Point_wsp_type* src = &points_wsp[srcIndex];
    Point_wsp_type* dest = &points_wsp[destIndex];

    std::vector<int> path;
    path.push_back(srcIndex);

    Iso_rectangle_2 biggest_box = src->rep_biggest_box();
    Point_wsp_type* point = src;
    while(biggest_box.bounded_side(*dest) == -1 && point != dest) {
      const std::vector<const Node*>& tos = point->rep_tos();

      //Verify if there is an edge from point to dest. If so, set biggest_box to the bounding box of the destination.
      for(Node_const_iterator it = tos.begin(); it != tos.end(); it++) {
        const Node* nodeTo = *it;
        if(nodeTo->bounding_box().bounded_side(*dest) != -1) {
          biggest_box = nodeTo->bounding_box();
          if(point->is_representative(nodeTo)) {
            point = dest;
          }
          else {
            point = node_representatives[nodeTo][0];
          }
          path.push_back(*point);
          goto destination_bbox_found;
        }
      }

      //Else, find the smallest bounding box from the neighbors of point such that it is bigger than biggest_box and such
      //that src is still in the box.
      Point_wsp_type* cur_point = NULL;
      Iso_rectangle_2 cur_box;
      for(Node_const_iterator toIt = tos.begin(); toIt != tos.end(); toIt++) {
        for(Point_wsp_const_iterator it = node_representatives[*toIt].begin(); it != node_representatives[*toIt].end(); it++) {
          Point_wsp_type* new_point = *it;

          const std::vector<const Node*>& froms = new_point->rep_froms();
          for(Node_const_iterator fromIt = froms.begin(); fromIt != froms.end(); fromIt++) {
            Iso_rectangle_2 new_box = (*fromIt)->bounding_box();
            if(new_box.bounded_side(*src) != -1 &&
                  biggest_box.area() < new_box.area() &&
                  (cur_point == NULL || new_box.area() < cur_box.area() || new_box.area() == cur_box.area())) {
              if(cur_point != NULL && new_box.area() == cur_box.area()) {
                if(cur_point->rep_biggest_box().area() < new_point->rep_biggest_box().area()) {
                  cur_point = new_point;
                  cur_box = new_box;
                }
              }
              else {
                cur_point = new_point;
                cur_box = new_box;
              }
            }
          }
        }
      }
      point = cur_point;
      biggest_box = cur_point->rep_biggest_box();
      path.push_back(*point);
    }

destination_bbox_found:
    while(point != dest) {
      const std::vector<const Node*>& tos = point->rep_tos();

      for(Node_const_iterator it = tos.begin(); it != tos.end(); it++) {
        const Node* nodeTo = *it;
        if(nodeTo->bounding_box().bounded_side(*dest) != -1 &&
              nodeTo->bounding_box().area() < biggest_box.area()) {
          biggest_box = nodeTo->bounding_box();
          if(dest->is_representative(nodeTo)) {
            point = dest;
          }
          else {
            point = node_representatives[nodeTo][0];
          }
          break;
        }
      }
      path.push_back(*point);
    }

    if(debug) {
      out << path << std::endl;
      out << (verify_algo_induction_proof(path, out, debug) ? "true" : "false") << std::endl;
    }
    return path;
  }

  bool verify_algo_induction_proof(std::vector<int> path) const {
    std::ostream cnull(NULL);
    return verify_algo_induction_proof(path, cnull, false);
  }

  bool verify_algo_induction_proof(std::vector<int> path, std::ostream& out, bool debug) const {
    if(debug) {
      out << "path: " << path << std::endl;
    }
    if(path.size() == 1) {
      return true;
    }

    Point_wsp_type* src = &points_wsp[path[0]];
    Point_wsp_type* dest = &points_wsp[path[path.size()-1]];

    const Well_separated_pair& pair = get_wsp(src, dest);
    if(debug) {
      out << "pair 1";
      print_node(out, pair.first);
      out << std::endl;
      out << "pair 2";
      print_node(out, pair.second);
      out << std::endl;
    }

    for(typename std::vector<int>::iterator it = path.begin(); it != path.end() - 1; it++) {
      if(is_pair_separating(pair, &points_wsp[*it], &points_wsp[*(it+1)])) {
        std::vector<int> pathA(path.begin(), it+1);
        std::vector<int> pathB(it+1, path.end());
        return verify_algo_induction_proof(pathA, out, debug) &&
                  verify_algo_induction_proof(pathB, out, debug);
      }
    }
    return false;
  }

  void display_wspd(std::ostream& out) const {
    for(int i = 0; i < this->points.size(); i++) {
      out << i << " " << this->points[i] << std::endl;
      Point_wsp_type* p = &points_wsp[i];
      const std::vector<Well_separated_pair>& rep = p->representative_of();
      for(typename std::vector<Well_separated_pair>::const_iterator it = rep.begin(); it != rep.end(); it++) {
        print_pair(out, *it);
        out << std::endl;
      }
    }
  }

private:
  const Well_separated_pair& get_wsp(Point_wsp_type* p1, Point_wsp_type* p2) const {
    const std::vector<Well_separated_pair>& pair = p1->pairs();
    for(typename std::vector<Well_separated_pair>::const_iterator it = pair.begin(); it != pair.end(); it++) {
      if(is_pair_separating(*it, p1, p2)) {
        return *it;
      }
    }
  }

  bool is_pair_separating(const Well_separated_pair& pair, Point_wsp_type* p1, Point_wsp_type* p2) const {
    const Node* node1 = pair.first;
    const Node* node2 = pair.second;
    return (node1->bounding_box().bounded_side(*p1) != -1 && node2->bounding_box().bounded_side(*p2) != -1) ||
        (node1->bounding_box().bounded_side(*p2) != -1 && node2->bounding_box().bounded_side(*p1) != -1);
  }

  void print_pair(std::ostream& out, const Well_separated_pair& pair) const {
    print_node(out, pair.first);
    out << " ";
    print_node(out, pair.second);
  }

  void print_node(std::ostream& out, const Node* node) const {
    out << "[";
    for(typename Point_container::const_iterator it = node->point_container().begin(); it != node->point_container().end(); it++) {
      if(it != node->point_container().begin()) {
        out << ", ";
      }
      out << point_indices[**it];
    }
    out << "]";
  }
private:
  mutable std::map<Point_2, int> point_indices;
  mutable std::vector<Point_wsp_type> points_wsp;
  mutable std::map<Point_2, Point_wsp_type*> points_to_points_wsp;
  mutable std::map<const Node*, std::vector<Point_wsp_type*> > node_representatives;
};

#endif // PATH_WSPD_H
