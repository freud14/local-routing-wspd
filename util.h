#ifndef UTIL_H
#define UTIL_H
#include <iostream>
#include <vector>

extern std::ostream cnull;

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

template <typename T>
std::ostream& operator<<(std::ostream& out, const std::vector<T*>& v)
{
  out << "[";
  for(size_t i = 0; i < v.size(); ++i) {
    if(i != 0)
      out << ", ";
    out << *v[i];
  }
  out << "]";
  return out;
}

template <typename T>
bool in_between(T in, T a, T b) {
  return (a <= in && in <= b) || (b <= in && in <= a);
}

#endif // UTIL_H
