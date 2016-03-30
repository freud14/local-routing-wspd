#ifndef UTIL_H
#define UTIL_H
#include <iostream>
#include <vector>

template <typename T>
std::ostream& operator<<(std::ostream& out, const std::vector<T>& v)
{
  out << "[";
  for(size_t i = 0; i < v.size(); ++i) {
    if(i != 0)
      out << ", ";
    out << v[i];
  }
  out << "]";
  return out;
}

#endif // UTIL_H
