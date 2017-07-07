

#ifndef CARTOGRAPHER_COMMON_MAKE_UNIQUE_H_
#define CARTOGRAPHER_COMMON_MAKE_UNIQUE_H_

#include <cstddef>
#include <memory>
#include <type_traits>
#include <utility>

namespace cartographer {
namespace common {

/*
make_unique.h在不支持c++14的环境下实现 std::make_unique的功能
实现细节:完美转发和移动语义

*/
// Implementation of c++14's std::make_unique, taken from
// https://isocpp.org/files/papers/N3656.txt
template <class T>
struct _Unique_if {
  typedef std::unique_ptr<T> _Single_object;
};

template <class T>
struct _Unique_if<T[]> {
  typedef std::unique_ptr<T[]> _Unknown_bound; //不支持数组(不定长)
};

template <class T, size_t N>
struct _Unique_if<T[N]> {                      //不支持数组(定长)
  typedef void _Known_bound;
};

template <class T, class... Args>
typename _Unique_if<T>::_Single_object make_unique(Args&&... args) {
  return std::unique_ptr<T>(new T(std::forward<Args>(args)...));  //完美转发参数
}

template <class T>
typename _Unique_if<T>::_Unknown_bound make_unique(size_t n) {
  typedef typename std::remove_extent<T>::type U;
  return std::unique_ptr<T>(new U[n]());
}

template <class T, class... Args>
typename _Unique_if<T>::_Known_bound make_unique(Args&&...) = delete;//不能使用定长数组

}  // namespace common
}  // namespace cartographer

#endif  // CARTOGRAPHER_COMMON_MAKE_UNIQUE_H_

/*
完美转发:
完美的传递函数参数而不修改参数类型，左值依然是左值，右值依然是右值。
*/