#ifndef BASE_SEARCH_COMPOSER_H
#define BASE_SEARCH_COMPOSER_H
#include <CGAL/WSPD.h>
#include "Point_wsp.h"

template <typename Traits>
class Search_filter_composer : public Base_search_filter<Traits>
{
public:
  typedef Point_wsp<Traits>                                            Point_wsp_type;
  typedef std::vector<Base_search_filter<Traits>*>                     Filter_vector;
  typedef typename std::vector<Base_search_filter<Traits>*>::iterator  Filter_iterator;
public:
  Search_filter_composer(Point_wsp_type* src_, Point_wsp_type* dest_) : Base_search_filter<Traits>(src_, dest_) { }

  virtual std::vector<Point_wsp_type*> filter(Point_wsp_type* point, const std::vector<Point_wsp_type*>& neighbors) {
    std::vector<Point_wsp_type*> ret(neighbors);
    for(Filter_iterator it = filters.begin(); it != filters.end(); it++) {
      Base_search_filter<Traits>* filter = *it;
      ret = filter->filter(point, ret);
      if(ret.size() == 1) {
        return ret;
      }
    }
    return ret;
  }

  Search_filter_composer<Traits>* compose(Base_search_filter<Traits>* filter) {
    filters.push_back(filter);
    return this;
  }

  virtual ~Search_filter_composer() {
    for(Filter_iterator it = filters.begin(); it != filters.end(); it++) {
      delete *it;
    }
  }

  virtual std::string to_string() const { return "Search_filter_composer"; }
private:
  Filter_vector filters;
};

#endif // BASE_SEARCH_COMPOSER_H
