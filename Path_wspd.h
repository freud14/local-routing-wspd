#ifndef PATH_WSPD_H
#define PATH_WSPD_H
#include <iostream>
#include "util.h"
#include <CGAL/Split_tree.h>
#include <CGAL/WSPD.h>
#include <map>
#include "Point_wsp.h"

typedef struct {
  bool biggest_box;
  bool edge_inside;
} Path_parameters;

template <typename Traits>
class Path_wspd : public CGAL::WSPD<Traits>
{
private:
  typedef typename Traits::K                                      K;
  typedef typename K::Point_2                                     Point_2;
  typedef typename K::Circle_2                                    Circle_2;
  typedef typename K::Iso_rectangle_2                             Iso_rectangle_2;
  typedef typename K::Segment_2                                   Segment_2;
  typedef typename K::Direction_2                                 Direction_2;

  typedef typename CGAL::WSPD<Traits>                             WSPD;
public:
  typedef typename WSPD::FT                                       FT;
  typedef typename WSPD::Split_tree::Point_container              Point_container;
  typedef typename WSPD::Node                                     Node;
  typedef typename WSPD::Well_separated_pair                      Well_separated_pair;
  typedef typename WSPD::Well_separated_pair_iterator             Well_separated_pair_iterator;
  typedef typename WSPD::Point_iterator                           Point_iterator;

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
        const Well_separated_pair& pair = *it;
        const Node* node1 = pair.first;
        const Node* node2 = pair.second;
        compute_points_wsp(pair, node1, node2);
        compute_points_wsp(pair, node2, node1);
      }
    }
  }

  void compute_points_wsp(const Well_separated_pair& pair, const Node* from, const Node* to) const {
    for(typename Point_container::const_iterator it = from->point_container().begin(); it != from->point_container().end(); it++) {
      Point_2 p = **it;
      points_to_points_wsp[p]->add_pair(pair, from, to);
      if(points_to_points_wsp[p]->is_representative(from)) {
        if(std::find(node_representatives[from].begin(), node_representatives[from].end(), points_to_points_wsp[p]) == node_representatives[from].end()) {
          node_representatives[from].push_back(points_to_points_wsp[p]);
        }
      }
    }
  }

  std::vector<int> find_path(int srcIndex, int destIndex, Path_parameters params) const {
    std::ostream cnull(NULL);
    return find_path(srcIndex, destIndex, cnull, false, false, params);
  }

  std::vector<int> find_path(int srcIndex, int destIndex, bool with_exception, Path_parameters params) const {
    std::ostream cnull(NULL);
    return find_path(srcIndex, destIndex, cnull, false, with_exception, params);
  }

  std::vector<int> find_path(int srcIndex, int destIndex, std::ostream& out, bool debug, bool with_exception, Path_parameters params) const {
    compute();
    Point_wsp_type* src = &points_wsp[srcIndex];
    Point_wsp_type* dest = &points_wsp[destIndex];

    std::vector<int> path;
    path.push_back(srcIndex);

    Point_wsp_type* point = src;
    Iso_rectangle_2 biggest_box = point->rep_biggest_box();
    while(biggest_box.bounded_side(*dest) == -1 && point != dest) {
      const std::vector<const Node*>& tos = point->rep_tos();

      //Verify if there is an edge from point to dest. If so, set biggest_box to the bounding box of the destination.
      if(verify_if_edge_bbox_destination(point, dest, biggest_box, path)) break;

      //Else, find the smallest bounding box from the neighbors of point such that it is bigger than biggest_box and such
      //that src is still in the box.
      Point_wsp_type* cur_point = NULL;
      Iso_rectangle_2 cur_box(Point_2(0,0), Point_2(0,0));
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
                if(params.biggest_box && cur_point->rep_biggest_box().area() < new_point->rep_biggest_box().area()) {
                  cur_point = new_point;
                  cur_box = new_box;
                }
                else if(cur_point->rep_biggest_box().area() == new_point->rep_biggest_box().area() || !params.biggest_box) {
                  if(params.edge_inside && biggest_box.bounded_side(*new_point) != -1) {
                    cur_point = new_point;
                    cur_box = new_box;
                  }
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
      biggest_box = point->rep_biggest_box();
      path.push_back(*point);
    }

    find_dest_in_its_bbox(point, dest, biggest_box, path);

    if(debug) {
      out << path << std::endl;
      out << (verify_algo_induction_proof(path, out, debug, with_exception) ? "true" : "false") << std::endl;
    }
    return path;
  }

  bool verify_if_edge_bbox_destination(Point_wsp_type*& point, Point_wsp_type* dest, Iso_rectangle_2& biggest_box, std::vector<int>& path) const {
    const std::vector<const Node*>& tos = point->rep_tos();
    for(Node_const_iterator it = tos.begin(); it != tos.end(); it++) {
      const Node* nodeTo = *it;
      if(nodeTo->bounding_box().bounded_side(*dest) != -1) {
        biggest_box = nodeTo->bounding_box();
        if(dest->is_representative(nodeTo)) {
          point = dest;
        }
        else {
          point = node_representatives[nodeTo][0];
        }
        path.push_back(*point);
        return true;
      }
    }
    return false;
  }

  void find_dest_in_its_bbox(Point_wsp_type*& point, Point_wsp_type* dest, Iso_rectangle_2& biggest_box, std::vector<int>& path) const {
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
  }

  bool verify_algo_induction_proof(std::vector<int> path) const {
    std::ostream cnull(NULL);
    return verify_algo_induction_proof(path, cnull, false, false);
  }

  bool verify_algo_induction_proof(std::vector<int> path, bool with_exception) const {
    std::ostream cnull(NULL);
    return verify_algo_induction_proof(path, cnull, false, with_exception);
  }

  bool verify_algo_induction_proof(std::vector<int> path, std::ostream& out, bool debug, bool with_exception) const {
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
        return verify_algo_induction_proof(pathA, out, debug, with_exception) &&
                  verify_algo_induction_proof(pathB, out, debug, with_exception);
      }
    }

    if(with_exception) {
      const Node* from = pair.first->bounding_box().bounded_side(*src) != -1 ? pair.first : pair.second;
      const Node* to = pair.first->bounding_box().bounded_side(*dest) != -1 ? pair.first : pair.second;
      std::vector<int> newPath;
      int i = 0;
      while(i < path.size() && from->bounding_box().bounded_side(this->points[path[i]]) != -1) {
        newPath.push_back(path[i]);
        i++;
      }
      int count = 0;
      while(i < path.size() && from->bounding_box().bounded_side(this->points[path[i]]) == -1 && to->bounding_box().bounded_side(this->points[path[i]]) == -1) {
        i++;
        count++;
      }
      newPath.insert(newPath.end(), path.begin()+i, path.end());

      if(count > 1) return false;
      else return verify_algo_induction_proof(newPath, out, debug, with_exception);
    }
    else {
      return false;
    }
  }

  std::vector<int> find_t_path(std::vector<int> path) const {
    std::vector<int> t_path;
    find_t_path(path, t_path);
    return t_path;
  }

  void find_t_path(std::vector<int> path, std::vector<int>& t_path) const {
    if(path.size() == 1) {
      t_path.push_back(path[0]);
      return;
    }

    Point_wsp_type* src = &points_wsp[path[0]];
    Point_wsp_type* dest = &points_wsp[path[path.size()-1]];

    const Well_separated_pair& pair = get_wsp(src, dest);
    for(typename std::vector<int>::iterator it = path.begin(); it != path.end() - 1; it++) {
      if(is_pair_separating(pair, &points_wsp[*it], &points_wsp[*(it+1)])) {
        std::vector<int> pathA(path.begin(), it+1);
        std::vector<int> pathB(it+1, path.end());
        find_t_path(pathA, t_path);
        find_t_path(pathB, t_path);
        return;
      }
    }

    const Node* from = pair.first->bounding_box().bounded_side(*src) != -1 ? pair.first : pair.second;
    const Node* to = pair.first->bounding_box().bounded_side(*dest) != -1 ? pair.first : pair.second;
    std::vector<int> newPath;
    int i = 0;
    while(i < path.size() && from->bounding_box().bounded_side(this->points[path[i]]) != -1) {
      newPath.push_back(path[i]);
      i++;
    }
    if(std::find(node_representatives[from].begin(), node_representatives[from].end(), &points_wsp[path[i-1]]) == node_representatives[from].end()) {
      newPath.push_back(*node_representatives[from][0]);
    }
    while(i < path.size() && from->bounding_box().bounded_side(this->points[path[i]]) == -1 && to->bounding_box().bounded_side(this->points[path[i]]) == -1) {
      i++;
    }
    if(std::find(node_representatives[to].begin(), node_representatives[to].end(), &points_wsp[path[i]]) == node_representatives[to].end()) {
      newPath.push_back(*node_representatives[to][0]);
    }
    newPath.insert(newPath.end(), path.begin()+i, path.end());
    find_t_path(newPath, t_path);
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

  const Well_separated_pair& get_wsp(int p1, int p2) const {
    return get_wsp(&points_wsp[p1], &points_wsp[p2]);
  }

  std::vector<int> get_points(const Node* node) {
    std::vector<int> ret;
    for(typename Point_container::const_iterator it = node->point_container().begin(); it != node->point_container().end(); it++) {
      ret.push_back(point_indices[**it]);
    }
    return ret;
  }

  std::vector<int> get_neighbors(int p) {
    std::vector<int> ret;
    Point_wsp_type* point = &points_wsp[p];
    const std::vector<const Node*>& tos = point->rep_tos();
    for(Node_const_iterator toIt = tos.begin(); toIt != tos.end(); toIt++) {
      for(Point_wsp_const_iterator it = node_representatives[*toIt].begin(); it != node_representatives[*toIt].end(); it++) {
        ret.push_back(**it);
      }
    }
    return ret;
  }
private:
  const Well_separated_pair& get_wsp(Point_wsp_type* p1, Point_wsp_type* p2) const {
    const std::vector<Well_separated_pair>& pair = p1->pairs();
    for(typename std::vector<Well_separated_pair>::const_iterator it = pair.begin(); it != pair.end(); it++) {
      if(is_pair_separating(*it, p1, p2)) {
        return *it;
      }
    }
    CGAL_assertion(false);
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
