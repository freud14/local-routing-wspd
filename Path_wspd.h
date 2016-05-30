#ifndef PATH_WSPD_H
#define PATH_WSPD_H
#include <iostream>
#include <algorithm>
#include <boost/shared_ptr.hpp>
#include "util.h"
#include <CGAL/enum.h>
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
  int bounded_biggest_box;
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
  typedef typename WSPD::Node::Point_iterator                     Node_iterator;
  typedef typename WSPD::Node_const_handle                        Node_const_handle;
  typedef typename WSPD::Well_separated_pair                      Well_separated_pair;
  typedef typename WSPD::Well_separated_pair_iterator             Well_separated_pair_iterator;
  typedef typename WSPD::Point_iterator                           Point_iterator;

private:
  typedef Point_wsp<Traits>                                       Point_wsp_type;
  typedef WSP_iterator<Traits>                                    WSP_iterator_type;
  typedef typename std::vector<Node_const_handle>::const_iterator Node_const_iterator;
  typedef typename std::vector<Point_wsp_type*>::const_iterator   Point_wsp_const_iterator;

  typedef boost::shared_ptr<Base_search_filter<Traits> >          Filter_ptr;
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

      if(this->split_tree().root() != NULL) {
        compute_tree_nodes_representatives(this->split_tree().root());
        for(Well_separated_pair_iterator it = this->wspd_begin(); it != this->wspd_end(); it++) {
          const Well_separated_pair& pair = *it;
          Node_const_handle node1 = pair.a();
          Node_const_handle node2 = pair.b();
          for(int i = 0; i < node_representatives[node1].size(); i++) {
            node_representatives[node1][i]->add_representative_of(pair, node1, node2);
          }
          for(int i = 0; i < node_representatives[node2].size(); i++) {
            node_representatives[node2][i]->add_representative_of(pair, node2, node1);
          }
        }
      }
    }
  }

  std::vector<Point_wsp_type*> compute_tree_nodes_representatives(Node_const_handle node) const {
    std::vector<Point_wsp_type*> current_boundary;
    if(node->is_leaf()) {
      Point_2 p = *node->points_begin();
      current_boundary.push_back(points_to_points_wsp[p]);
    }
    else {
      std::vector<Point_wsp_type*> left_boundary = compute_tree_nodes_representatives(node->left());
      std::vector<Point_wsp_type*> right_boundary = compute_tree_nodes_representatives(node->right());
      typedef typename std::vector<Point_wsp_type*>::iterator point_it;
      for(point_it it = left_boundary.begin(); it != left_boundary.end(); it++) {
        if((*it)->is_representative(node)) {
          current_boundary.push_back(*it);
        }
      }
      for(point_it it = right_boundary.begin(); it != right_boundary.end(); it++) {
        if((*it)->is_representative(node)) {
          current_boundary.push_back(*it);
        }
      }
    }
    node_representatives[node].assign(current_boundary.begin(), current_boundary.end());
    return current_boundary;
  }

  std::vector<int> find_path(int srcIndex, int destIndex, Path_parameters params, bool with_exception = false, std::ostream& out = cnull, bool debug = false) const {
    compute();
    Point_wsp_type* src = &points_wsp[srcIndex];
    Point_wsp_type* dest = &points_wsp[destIndex];
    Filter_ptr filter = get_filter(params, src, dest);

    std::vector<int> old_candidates;
    std::vector<int> new_candidates;

    std::vector<int> path;
    path.push_back(srcIndex);

    Point_wsp_type* point = src;
    Iso_rectangle_2 biggest_box = point->rep_biggest_box();
    while(true) { //dest->is_outside(biggest_box) && point != dest) {
      //Verify if there is an edge from point to dest. If so, set biggest_box to the bounding box of the destination.
      if(verify_if_edge_bbox_destination(point, src, dest, biggest_box, path)) break;
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
        return path;
      }
      point = candidates[0];
      biggest_box = point->rep_biggest_box();
      path.push_back(*point);
    }

    find_dest_in_its_bbox(point, dest, biggest_box, path);

    if(debug) {
      out << path << std::endl;
      int nb_exception = 0;
      out << std::boolalpha << verify_algo_induction_proof(path, with_exception, nb_exception, out, debug) << std::noboolalpha << std::endl;
      if(with_exception) out << "Nb exceptions: " << nb_exception << std::endl;
    }

    return path;
  }

  bool verify_if_two_edges_bbox_destination(Point_wsp_type*& point, Point_wsp_type* dest, Iso_rectangle_2& biggest_box, std::vector<int>& path) const {
    Iso_rectangle_2 next_biggest_box;
    std::pair<Point_wsp_type*, Point_wsp_type*> next;
    next.first = NULL;
    next.second = NULL;

    for(WSP_iterator_type pairIt = point->representative_of_begin(); pairIt != point->representative_of_end(); pairIt++) {
      Node_const_handle nodeTo = pairIt.to();
      for(Point_wsp_const_iterator it = node_representatives[nodeTo].begin(); it != node_representatives[nodeTo].end(); it++) {
        if(next.first == NULL || CGAL::squared_distance(point->point(), (*it)->point()) < CGAL::squared_distance(point->point(), next.first->point())) {
          for(WSP_iterator_type pairIt2 = (*it)->representative_of_begin(); pairIt2 != (*it)->representative_of_end(); pairIt2++) {
            Node_const_handle nodeTo2 = pairIt2.to();
            if(dest->is_inside(nodeTo2->bounding_box())) {
              next_biggest_box = nodeTo2->bounding_box();
              if(dest->is_representative(nodeTo)) {
                next.second = dest;
              }
              else {
                next.second = node_representatives[nodeTo2][0];
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

  bool verify_if_edge_bbox_destination(Point_wsp_type*& point, Point_wsp_type* src, Point_wsp_type* dest, Iso_rectangle_2& biggest_box, std::vector<int>& path) const {
    Iso_rectangle_2 new_biggest_box = biggest_box;
    Point_wsp_type* new_point = NULL;
    Node_const_handle new_from;
    for(WSP_iterator_type pairIt = point->representative_of_begin(); pairIt != point->representative_of_end(); pairIt++) {
      Node_const_handle nodeTo = pairIt.to();
      if(dest->is_inside(nodeTo->bounding_box())) {
        new_from = pairIt.from();
        new_biggest_box = nodeTo->bounding_box();
        if(dest->is_representative(nodeTo)) {
          new_point = dest;
        }
        else {
          new_point = node_representatives[nodeTo][0];
        }
        break;
      }
    }

    if(new_point == NULL) return false;
    else {
      if(src->is_outside(new_from->bounding_box())) {
        for(WSP_iterator_type pairIt = point->representative_of_begin(); pairIt != point->representative_of_end(); pairIt++) {
          Node_const_handle nodeTo = pairIt.to();
          for(Point_wsp_const_iterator it = node_representatives[nodeTo].begin(); it != node_representatives[nodeTo].end(); it++) {
            Point_wsp_type* new_possible_rep = *it;
            for(WSP_iterator_type nprIt = new_possible_rep->representative_of_begin(); nprIt != new_possible_rep->representative_of_end(); nprIt++) {
              Node_const_handle nprNodeFrom = nprIt.from();
              Node_const_handle nprNodeTo = nprIt.to();
              if(src->is_inside(nprNodeFrom->bounding_box()) && dest->is_inside(nprNodeTo->bounding_box())) {
                path.push_back(*new_possible_rep);

                if(dest->is_representative(nprNodeTo)) {
                  point = dest;
                }
                else {
                  point = node_representatives[nprNodeTo][0];
                }
                path.push_back(*point);
                biggest_box = nprNodeTo->bounding_box();

                return true;
              }
            }
          }
        }
      }

      point = new_point;
      biggest_box = new_biggest_box;
      path.push_back(*point);
      return true;
    }
  }

  void find_dest_in_its_bbox(Point_wsp_type*& point, Point_wsp_type* dest, Iso_rectangle_2& biggest_box, std::vector<int>& path) const {
    while(point != dest) {
      for(WSP_iterator_type pairIt = point->representative_of_begin(); pairIt != point->representative_of_end(); pairIt++) {
        Node_const_handle nodeTo = pairIt.to();
        if(dest->is_inside(nodeTo->bounding_box()) &&
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

  bool verify_algo_induction_proof(std::vector<int> path, bool with_exception = false, int& nb_exception = 0, std::ostream& out = cnull, bool debug = false) const {
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
        return verify_algo_induction_proof(pathA, with_exception, nb_exception, out, debug) &&
                  verify_algo_induction_proof(pathB, with_exception, nb_exception, out, debug);
      }
    }

    if(with_exception) {
      nb_exception++;
      Node_const_handle from = src->is_inside(pair.a()->bounding_box()) ? pair.a() : pair.b();
      Node_const_handle to = dest->is_inside(pair.a()->bounding_box()) ? pair.a() : pair.b();
      std::vector<int> newPath;
      int i = 0;
      while(i < path.size() && points_wsp[path[i]].is_inside(from->bounding_box())) {
        newPath.push_back(path[i]);
        i++;
      }
      int count = 0;
      while(i < path.size() && points_wsp[path[i]].is_outside(from->bounding_box()) && points_wsp[path[i]].is_outside(to->bounding_box())) {
        i++;
        count++;
      }
      newPath.insert(newPath.end(), path.begin()+i, path.end());

      if(count > 1) return false;
      else return verify_algo_induction_proof(newPath, with_exception, nb_exception, out, debug);
    }
    else {
      return false;
    }
  }

  std::vector<int> find_t_path(std::vector<int> path, Path_parameters params) const {
    std::vector<int> t_path;
    find_t_path(path, t_path, params);
    return t_path;
  }

  void find_t_path(std::vector<int> path, std::vector<int>& t_path, Path_parameters params) const {
    if(path.size() == 1) {
      t_path.push_back(path[0]);
      return;
    }

    Point_wsp_type* src = &points_wsp[path[0]];
    Point_wsp_type* dest = &points_wsp[path[path.size()-1]];
    Filter_ptr filter = get_filter(params, src, dest, true);

    const Well_separated_pair& pair = get_wsp(src, dest);
    for(typename std::vector<int>::iterator it = path.begin(); it != path.end() - 1; it++) {
      if(is_pair_separating(pair, &points_wsp[*it], &points_wsp[*(it+1)])) {
        std::vector<int> pathA(path.begin(), it+1);
        std::vector<int> pathB(it+1, path.end());
        find_t_path(pathA, t_path, params);
        find_t_path(pathB, t_path, params);
        return;
      }
    }

    Node_const_handle from = src->is_inside(pair.a()->bounding_box()) ? pair.a() : pair.b();
    Node_const_handle to = dest->is_inside(pair.a()->bounding_box()) ? pair.a() : pair.b();
    std::vector<int> newPath;
    int i = 0;
    while(i < path.size() && points_wsp[path[i]].is_inside(from->bounding_box())) {
      newPath.push_back(path[i]);
      i++;
    }
    Point_wsp_type* p = &points_wsp[path[i-1]];
    while(i < path.size() && points_wsp[path[i]].is_outside(from->bounding_box()) && points_wsp[path[i]].is_outside(to->bounding_box())) {
      i++;
    }
    Point_wsp_type* q = &points_wsp[path[i]];
    newPath.pop_back();
    find_t_path(p, q, params, newPath);
    newPath.insert(newPath.end(), path.begin()+i+1, path.end());
    find_t_path(newPath, t_path, params);
  }

  std::vector<int> find_t_path(int srcIndex, int destIndex, Path_parameters params) const {
    Point_wsp_type* src = &points_wsp[srcIndex];
    Point_wsp_type* dest = &points_wsp[destIndex];
    std::vector<int> t_path;
    find_t_path(src, dest, params, t_path);
    return t_path;
  }

  void find_t_path(Point_wsp_type* src, Point_wsp_type* dest, Path_parameters params, std::vector<int>& t_path) const {
    if(src == dest) {
      t_path.push_back(*src);
      return;
    }

    Filter_ptr filter = get_filter(params, src, dest, true);
    Well_separated_pair pair = get_wsp(src, dest);
    Node_const_handle from = src->is_inside(pair.a()->bounding_box()) ? pair.a() : pair.b();
    Node_const_handle to = dest->is_inside(pair.a()->bounding_box()) ? pair.a() : pair.b();

    Point_wsp_type* p = filter->filter(src, node_representatives[from])[0];
    if(std::find(node_representatives[from].begin(), node_representatives[from].end(), src) != node_representatives[from].end()) {
      p = src;
    }
    find_t_path(src, p, params, t_path);

    Point_wsp_type* q = node_representatives[to][0];
    if(std::find(node_representatives[to].begin(), node_representatives[to].end(), dest) != node_representatives[to].end()) {
      q = dest;
    }
    find_t_path(q, dest, params, t_path);
  }


  void display_wspd(std::ostream& out) const {
    for(int i = 0; i < this->points.size(); i++) {
      out << i << " " << this->points[i] << std::endl;
      Point_wsp_type* p = &points_wsp[i];
      for(WSP_iterator_type it = p->representative_of_begin(); it != p->representative_of_end(); it++) {
        print_pair(out, *it);
        out << std::endl;
      }
    }
  }

  Well_separated_pair get_wsp(int p1, int p2) const {
    return get_wsp(&points_wsp[p1], &points_wsp[p2]);
  }

  std::vector<int> get_points(Node_const_handle node) const {
    std::vector<int> ret;
    for(Node_iterator it = node->points_begin(); it != node->points_end(); it++) {
      ret.push_back(point_indices[*it]);
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
    for(WSP_iterator_type pairIt = point->representative_of_begin(); pairIt != point->representative_of_end(); pairIt++) {
      Node_const_handle from = pairIt.from();
      if(std::find(ret.begin(), ret.end(), from->bounding_box()) == ret.end()) {
        ret.push_back(from->bounding_box());
      }
    }
    return ret;
  }

  std::vector<int> get_candidates(Path_parameters params, int src, int dest, int cur_point) const {
    Point_wsp_type* point = &points_wsp[cur_point];
    std::vector<Point_wsp_type*> neighbors = get_neighbors(point);
    Filter_ptr filter = get_filter(params, &points_wsp[src], &points_wsp[dest]);
    std::vector<Point_wsp_type*> candidates = filter->filter(point, neighbors);
    std::vector<int> ret;
    for(int i = 0; i < candidates.size(); i++) {
      ret.push_back(*candidates[i]);
    }
    return ret;
  }

  Filter_ptr get_filter(Path_parameters params, Point_wsp_type* src, Point_wsp_type* dest, bool tpath = false) const {
    std::vector<std::pair<int, Filter_ptr> > filters;

    #define ADD_FILTER(param_name, Class_name) \
    if(params.param_name != 0 && !tpath) { \
      filters.push_back(std::make_pair(params.param_name, new Class_name<Traits>(src, dest))); \
    }

    #define ADD_FILTER_ARGS(param_name, Class_name, args...) \
    if(params.param_name != 0 && !tpath) { \
      filters.push_back(std::make_pair(params.param_name, new Class_name<Traits>(src, dest, args))); \
    }

    #define ADD_FILTER_TPATH(param_name, Class_name) \
    if(params.param_name != 0) { \
      filters.push_back(std::make_pair(params.param_name, new Class_name<Traits>(src, dest))); \
    }

    ADD_FILTER(bigger_smallest_bbox, Bigger_smallest_bbox_filter);
    ADD_FILTER(biggest_box_ws, Biggest_bbox_ws_filter);
    ADD_FILTER(biggest_box, Biggest_bbox_filter);
    ADD_FILTER_ARGS(bounded_biggest_box, Bounded_biggest_bbox_filter, this->separation_ratio());
    ADD_FILTER(edge_inside, Edge_inside_filter);
    ADD_FILTER_TPATH(direction, Direction_filter);
    ADD_FILTER_TPATH(monotone_x, X_monotone_filter);
    ADD_FILTER_ARGS(src_inside, Src_inside_filter, node_representatives);
    ADD_FILTER_TPATH(euclidean, Euclidean_filter);

    std::sort(filters.begin(), filters.end());

    Search_filter_composer<Traits>* filter = new Search_filter_composer<Traits>(src, dest);
    for(int i = 0; i < filters.size(); i++) {
      filter->compose(filters[i].second);
    }
    return Filter_ptr(filter);
  }

  std::vector<Well_separated_pair> get_representative_pairs(int p) const {
    return std::vector<Well_separated_pair>(points_wsp[p].representative_of_begin(), points_wsp[p].representative_of_end());
  }
private:
  Well_separated_pair get_wsp(Point_wsp_type* p1, Point_wsp_type* p2) const {
    Node_const_handle v = this->split_tree().root()->left();
    Node_const_handle w = this->split_tree().root()->right();
    bool lca_found = false;
    Node_const_handle lca = this->split_tree().root();
    while(!lca_found) {
      if(p1->is_inside(v->bounding_box()) && p2->is_inside(v->bounding_box())) {
        lca = v;
      }
      else if(p1->is_inside(w->bounding_box()) && p2->is_inside(w->bounding_box())) {
        lca = w;
      }
      else {
        lca_found = true;
      }
      v = lca->left();
      w = lca->right();
    }

    // Assumes that p1 is in v and p2 in w
    bool swapped = false;
    if(p1->is_inside(w->bounding_box())) {
      std::swap(v, w);
      swapped = true;
    }

    while(!v->is_well_separated_with(w, this->s)) {
      if(v->has_longuer_side_than(w)) {
        if(p1->is_inside(v->left()->bounding_box())) {
          v = v->left();
        }
        else {
          v = v->right();
        }
      }
      else {
        if(p2->is_inside(w->left()->bounding_box())) {
          w = w->left();
        }
        else {
          w = w->right();
        }
      }
    }

    if(swapped) std::swap(v, w);

    return Well_separated_pair(v, w);
  }

  bool is_pair_separating(const Well_separated_pair& pair, Point_wsp_type* p1, Point_wsp_type* p2) const {
    Node_const_handle node1 = pair.a();
    Node_const_handle node2 = pair.b();

    return (p1->is_inside(node1->bounding_box()) && p2->is_inside(node2->bounding_box())) ||
        (p2->is_inside(node1->bounding_box()) && p1->is_inside(node2->bounding_box()));
  }

  std::vector<Point_wsp_type*> get_neighbors(Point_wsp_type* point) const {
    std::vector<Point_wsp_type*> ret;
    for(WSP_iterator_type pairIt = point->representative_of_begin(); pairIt != point->representative_of_end(); pairIt++) {
      Node_const_handle nodeTo = pairIt.to();
      for(Point_wsp_const_iterator it = node_representatives[nodeTo].begin(); it != node_representatives[nodeTo].end(); it++) {
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

  void print_node(std::ostream& out, Node_const_handle node) const {
    out << "[";
    for(Node_iterator it = node->points_begin(); it != node->points_end(); it++) {
      if(it != node->points_begin()) {
        out << ", ";
      }
      out << point_indices[*it];
    }
    out << "]";
  }
private:
  mutable std::map<Point_2, int> point_indices;
  mutable std::vector<Point_wsp_type> points_wsp;
  mutable std::map<Point_2, Point_wsp_type*> points_to_points_wsp;
  mutable std::map<Node_const_handle, std::vector<Point_wsp_type*> > node_representatives;
};

#endif // PATH_WSPD_H
