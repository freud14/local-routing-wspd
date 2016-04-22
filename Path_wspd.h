#ifndef PATH_WSPD_H
#define PATH_WSPD_H
#include <iostream>
#include "util.h"
#include <CGAL/Split_tree.h>
#include <CGAL/WSPD.h>
#include <map>
#include "Point_wsp.h"
#include "WSP_iterator.h"
#include "Search_filter/filters.h"


typedef struct {
  bool watch_2_edges;
  int bigger_smallest_bbox;
  int biggest_box_ws;
  int biggest_box;
  int edge_inside;
  int direction;
  int monotone_x;
  int src_inside;
  int euclidean;
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
  typedef WSP_iterator<Traits>                                    WSP_iterator_type;
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
        const Node* node1 = pair.a();
        const Node* node2 = pair.b();
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

private:
  static std::ostream cnull;
public:

  std::vector<int> find_path(int srcIndex, int destIndex, Path_parameters params, bool with_exception = false, std::ostream& out = cnull, bool debug = false) const {
    compute();
    Point_wsp_type* src = &points_wsp[srcIndex];
    Point_wsp_type* dest = &points_wsp[destIndex];
    Base_search_filter<Traits>* filter = get_filter(params, src, dest);

    std::vector<int> old_candidates;
    std::vector<int> new_candidates;

    std::vector<int> path;
    path.push_back(srcIndex);

    Point_wsp_type* point = src;
    Iso_rectangle_2 biggest_box = point->rep_biggest_box();
    while(biggest_box.bounded_side(*dest) == -1 && point != dest) {
      const std::vector<const Node*>& tos = point->rep_tos();

      //Verify if there is an edge from point to dest. If so, set biggest_box to the bounding box of the destination.
      if(verify_if_edge_bbox_destination(point, dest, biggest_box, path)) break;
      if(params.watch_2_edges && verify_if_two_edges_bbox_destination(point, dest, biggest_box, path)) break;

      //Else, find the smallest bounding box from the neighbors of point such that it is bigger than biggest_box and such
      //that src is still in the box.
      std::vector<Point_wsp_type*> neighbors = get_neighbors(point);
      std::vector<Point_wsp_type*> candidates = filter->filter(point, neighbors);
      if(candidates.size() == 0) {
        if(debug) {
          out << path << std::endl;
          out << "No candidate found!" << std::endl;
        }
        goto clean_exit;
      }
      point = candidates[0];
      biggest_box = point->rep_biggest_box();
      path.push_back(*point);
    }

    find_dest_in_its_bbox(point, dest, biggest_box, path);

    if(debug) {
      out << path << std::endl;
      out << (verify_algo_induction_proof(path, with_exception, out, debug) ? "true" : "false") << std::endl;
    }

clean_exit:
    delete filter;
    return path;
  }

  bool verify_if_two_edges_bbox_destination(Point_wsp_type*& point, Point_wsp_type* dest, Iso_rectangle_2& biggest_box, std::vector<int>& path) const {
    Iso_rectangle_2 next_biggest_box;
    std::pair<Point_wsp_type*, Point_wsp_type*> next;
    next.first = NULL;
    next.second = NULL;

    const std::vector<const Node*>& tos = point->rep_tos();
    for(Node_const_iterator toIt = tos.begin(); toIt != tos.end(); toIt++) {
      for(Point_wsp_const_iterator it = node_representatives[*toIt].begin(); it != node_representatives[*toIt].end(); it++) {
        if(next.first == NULL || CGAL::squared_distance(point->point(), (*it)->point()) < CGAL::squared_distance(point->point(), next.first->point())) {
          for(WSP_iterator_type pairIt = WSP_iterator_type::begin(**it); pairIt != WSP_iterator_type::end(**it); pairIt++) {
            const Node* nodeTo = pairIt.to();
            if(nodeTo->bounding_box().bounded_side(*dest) != -1) {
              next_biggest_box = nodeTo->bounding_box();
              if(dest->is_representative(nodeTo)) {
                next.second = dest;
              }
              else {
                next.second = node_representatives[nodeTo][0];
              }
              next.first = *it;
            }
          }
        }
      }
    }

    if(next.first != NULL) {
      path.push_back(*next.first);
      path.push_back(*next.second);
      point = next.second;
      biggest_box = next_biggest_box;
      return true;
    }
    else {
      return false;
    }

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

  bool verify_algo_induction_proof(std::vector<int> path, bool with_exception = false, std::ostream& out = cnull, bool debug = false) const {
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
      print_node(out, pair.a());
      out << std::endl;
      out << "pair 2";
      print_node(out, pair.b());
      out << std::endl;
    }

    for(typename std::vector<int>::iterator it = path.begin(); it != path.end() - 1; it++) {
      if(is_pair_separating(pair, &points_wsp[*it], &points_wsp[*(it+1)])) {
        std::vector<int> pathA(path.begin(), it+1);
        std::vector<int> pathB(it+1, path.end());
        return verify_algo_induction_proof(pathA, with_exception, out, debug) &&
                  verify_algo_induction_proof(pathB, with_exception, out, debug);
      }
    }

    if(with_exception) {
      const Node* from = pair.a()->bounding_box().bounded_side(*src) != -1 ? pair.a() : pair.b();
      const Node* to = pair.a()->bounding_box().bounded_side(*dest) != -1 ? pair.a() : pair.b();
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
      else return verify_algo_induction_proof(newPath, with_exception, out, debug);
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

    const Node* from = pair.a()->bounding_box().bounded_side(*src) != -1 ? pair.a() : pair.b();
    const Node* to = pair.a()->bounding_box().bounded_side(*dest) != -1 ? pair.a() : pair.b();
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

  std::vector<int> get_neighbors(int p) const {
    std::vector<int> ret;
    Point_wsp_type* point = &points_wsp[p];
    std::vector<Point_wsp_type*> pointRet = get_neighbors(point);
    for(int i = 0; i < pointRet.size(); i++) {
      ret.push_back(*pointRet[i]);
    }
    return ret;
  }


  std::vector<Iso_rectangle_2> get_bboxes(int p) {
    std::vector<Iso_rectangle_2> ret;
    Point_wsp_type* point = &points_wsp[p];
    const std::vector<const Node*>& froms = point->rep_froms();
    for(Node_const_iterator fromIt = froms.begin(); fromIt != froms.end(); fromIt++) {
      const Node* from = *fromIt;
      if(std::find(ret.begin(), ret.end(), from->bounding_box()) == ret.end()) {
        ret.push_back(from->bounding_box());
      }
    }
    return ret;
  }

  std::vector<int> get_candidates(Path_parameters params, int src, int dest, int cur_point) const {
    Point_wsp_type* point = &points_wsp[cur_point];
    std::vector<Point_wsp_type*> neighbors = get_neighbors(point);
    Base_search_filter<Traits>* filter = get_filter(params, &points_wsp[src], &points_wsp[dest]);
    std::vector<Point_wsp_type*> candidates = filter->filter(point, neighbors);
    delete filter;
    std::vector<int> ret;
    for(int i = 0; i < candidates.size(); i++) {
      ret.push_back(*candidates[i]);
    }
    return ret;
  }

  Base_search_filter<Traits>* get_filter(Path_parameters params, Point_wsp_type* src, Point_wsp_type* dest) const {
    std::vector<std::pair<int, Base_search_filter<Traits>* > > filters;

    #define ADD_FILTER(param_name, Class_name) \
    if(params.param_name != 0) { \
      filters.push_back(std::make_pair(params.param_name, new Class_name<Traits>(src, dest))); \
    }

    ADD_FILTER(bigger_smallest_bbox, Bigger_smallest_bbox_filter);
    ADD_FILTER(biggest_box_ws, Biggest_bbox_ws_filter);
    ADD_FILTER(biggest_box, Biggest_bbox_filter);
    ADD_FILTER(edge_inside, Edge_inside_filter);
    ADD_FILTER(direction, Direction_filter);
    ADD_FILTER(monotone_x, X_monotone_filter);
    ADD_FILTER(src_inside, Src_inside_filter);
    ADD_FILTER(euclidean, Euclidean_filter);

    std::sort(filters.begin(), filters.end());

    Search_filter_composer<Traits>* filter = new Search_filter_composer<Traits>(src, dest);
    for(int i = 0; i < filters.size(); i++) {
      filter->compose(filters[i].second);
    }
    return filter;
  }

  Point_wsp_type* get_point_wsp(int p) {
    return &points_wsp[p];
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
    const Node* node1 = pair.a();
    const Node* node2 = pair.b();
    return (node1->bounding_box().bounded_side(*p1) != -1 && node2->bounding_box().bounded_side(*p2) != -1) ||
        (node1->bounding_box().bounded_side(*p2) != -1 && node2->bounding_box().bounded_side(*p1) != -1);
  }

  std::vector<Point_wsp_type*> get_neighbors(Point_wsp_type* point) const {
    std::vector<Point_wsp_type*> ret;
    const std::vector<const Node*>& tos = point->rep_tos();
    for(Node_const_iterator toIt = tos.begin(); toIt != tos.end(); toIt++) {
      for(Point_wsp_const_iterator it = node_representatives[*toIt].begin(); it != node_representatives[*toIt].end(); it++) {
        if(std::find(ret.begin(), ret.end(), *it) == ret.end()) {
          ret.push_back(*it);
        }
      }
    }
    return ret;
  }

  void print_pair(std::ostream& out, const Well_separated_pair& pair) const {
    print_node(out, pair.a());
    out << " ";
    print_node(out, pair.b());
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

template <typename Traits>
std::ostream Path_wspd<Traits>::cnull(NULL);

#endif // PATH_WSPD_H
