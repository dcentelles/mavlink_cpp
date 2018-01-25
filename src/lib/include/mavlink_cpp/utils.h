#ifndef UTILS_H
#define UTILS_H

#include <cstdarg>
#include <cstring>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

namespace mavlink_cpp {

template <typename T> using Ptr = std::shared_ptr<T>;

template <typename T, typename... Targs>
static Ptr<T> CreateObject(Targs... Fargs) {
  Ptr<T> ptr(new T(Fargs...));
  return ptr;
}
std::string format(const std::string &format, ...) {
  va_list args;
  va_start(args, format);
  size_t len = std::vsnprintf(NULL, 0, format.c_str(), args);
  va_end(args);
  std::vector<char> vec(len + 1);
  va_start(args, format);
  std::vsnprintf(&vec[0], len + 1, format.c_str(), args);
  va_end(args);
  return &vec[0];
}
}
#endif // UTILS_H
