#ifndef CARTOGRAPHER_IO_CAIRO_TYPES_H_
#define CARTOGRAPHER_IO_CAIRO_TYPES_H_

#include <memory>

#include "cairo/cairo.h"
/*
Cairo是一个2D图形库，支持多种输出设备。
cairo 是一个免费的矢量绘图软件库，它可以绘制多种输出格式。

https://www.ibm.com/developerworks/cn/linux/l-cairo/
http://liyanrui.is-programmer.com/2009/3/18/cairo-tutorial-05.7742.html
http://blog.csdn.net/turingo/article/details/8131057

*/
namespace cartographer {
namespace io {
namespace cairo {

// std::unique_ptr for Cairo surfaces. The surface is destroyed when the
// std::unique_ptr is reset or destroyed.
using UniqueSurfacePtr =
    std::unique_ptr<cairo_surface_t, void (*)(cairo_surface_t*)>;

// std::unique_ptr for Cairo contexts. The context is destroyed when the
// std::unique_ptr is reset or destroyed.
using UniqueContextPtr = std::unique_ptr<cairo_t, void (*)(cairo_t*)>;

// std::unique_ptr for Cairo paths. The path is destroyed when the
// std::unique_ptr is reset or destroyed.
using UniquePathPtr = std::unique_ptr<cairo_path_t, void (*)(cairo_path_t*)>;

}  // namespace cairo
}  // namespace io
}  // namespace cartographer

#endif  // CARTOGRAPHER_IO_CAIRO_TYPES_H_
