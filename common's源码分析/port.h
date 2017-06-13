/*
boost::iostreams主要有两类东西组成，一个是device，另一个是filter，可以到源码目录下找，iostreams目录下有这两个目录可以找到相关类。
device像是一种设备，不能单独使用，要配合普通流stream或stream_buffer来使用，可将流中的数据输入/输出到这个设备上，可分为:
Source，它以读取的方式访问字符序列，如：file_source 做文件输入。
Sink，它以写的方式访问字符序列，如：file_sink 做文件输出。
stream<file_source> 那么这个就是一个文件输入流，类似于ifilestream，而stream<file_sink>就是一个文件输出流，类似于ofilestream。
filter像一种过滤器，和device一样，也是不能单独使用的，要配合过滤流filtering_stream或filtering_streambuf来使用，将流中的数据按一种规则过滤，可分为：
InputFilter，过滤通过Source读取的输入，如：gzip_decompressor 按gzip算法解压流中的数据。
OutputFilter，过滤向Sink写入的输出，如：gzip_compressor 按gzip算法压缩流中的数据。
但filtering_stream是要维护一个filter的链表的，以device为结束。输出过滤流filtering_ostream，是按顺序执行filter，然后再输出到devic上

如：
压缩时
filtering_ostream out;
out.push(gzip_compressor()); //gzip OutputFilter
out.push(bzip2_compressor());//bzip2 OutputFilter
out.push(boost::iostreams::file_sink("test.txt"));//以file_sink device结束
这就会将流的数据先按gzip压缩，然后再按bzip2压缩之后，才输出到text.txt文件中。

解压时
filtering_istream in;
in.push(gzip_decompressor());/gzip InputFilter
in.push(bzip2_decompressor());/bzip2 InputFilter
in.push(file_source("test.txt"));
这时候先将test.txt文件中数据读出，然后按bzip2解压，然后再按gzip解压，存入in流中，正好是压缩的逆序。
*/
#ifndef CARTOGRAPHER_COMMON_PORT_H_
#define CARTOGRAPHER_COMMON_PORT_H_

#include <cinttypes>//完成字节和宽字符串到std::intmax或std::uintmax的转换
#include <cmath>
#include <string>

#include <boost/iostreams/device/back_inserter.hpp>//一个成员函数，返回值是back_insert_iterator, 本质上是push_back进行操作的, 返回值back_insert_iterator, 并实现其自增.
/*
   

*/
#include <boost/iostreams/filter/gzip.hpp>//包含六的解压与压缩算法
#include <boost/iostreams/filtering_stream.hpp>//配合filter实现流过滤

using int8 = int8_t;
using int16 = int16_t;
using int32 = int32_t;
using int64 = int64_t;
using uint8 = uint8_t;
using uint16 = uint16_t;
using uint32 = uint32_t;
using uint64 = uint64_t;

using std::string;//采用using声明，把std ns的特定成员的标识符号加进来
/*
using 声明：更安全，减少变量和函数命名的冲突
using编译指令：(using namespace)把所有std命名空间下的成员加进来
*/
namespace cartographer {
namespace common {

inline int RoundToInt(const float x) { return std::lround(x); }//四舍五入运算

inline int RoundToInt(const double x) { return std::lround(x); }

inline int64 RoundToInt64(const float x) { return std::lround(x); }

inline int64 RoundToInt64(const double x) { return std::lround(x); }

inline void FastGzipString(const string& uncompressed, string* compressed) {
  boost::iostreams::filtering_ostream out;//创建过滤流对象
  out.push(
      boost::iostreams::gzip_compressor(boost::iostreams::zlib::best_speed));//绑定压缩算法，求其速度
  out.push(boost::iostreams::back_inserter(*compressed));//选择压缩流从后迭代插入模式
  boost::iostreams::write(out,
                          reinterpret_cast<const char*>(uncompressed.data()),
                          uncompressed.size());//实现uncompressed到compressed的单向流
}
/*
reinterpret_cast的转换格式：reinterpret_cast <type-id> (expression)
http://www.jellythink.com/archives/205">强制转换博客地址
允许将任何指针类型转换为其它的指针类型；听起来很强大，但是也很不靠谱。
它主要用于将一种数据类型从一种类型转换为另一种类型。它可以将一个指针转换成一个整数，
也可以将一个整数转换成一个指针，在实际开发中，先把一个指针转换成一个整数，
在把该整数转换成原类型的指针，还可以得到原来的指针值；特别是开辟了系统全局的内存空间，
需要在多个应用程序之间使用时，需要彼此共享，传递这个内存空间的指针时，就可以将指针转换成整数值，
得到以后，再将整数值转换成指针，进行对应的操作。
*/
inline void FastGunzipString(const string& compressed, string* decompressed) {
  boost::iostreams::filtering_ostream out;
  out.push(boost::iostreams::gzip_decompressor());
  out.push(boost::iostreams::back_inserter(*decompressed));
  boost::iostreams::write(out, reinterpret_cast<const char*>(compressed.data()),
                          compressed.size());//解压
}

}  // namespace common
}  // namespace cartographer

#endif  // CARTOGRAPHER_COMMON_PORT_H_