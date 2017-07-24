
源码可在https://github.com/learnmoreonce/SLAM 下载

``` c++

源码分析文件:common/port.h
 
/*
  common-port.h文件主要实现2大功能：
  1，使用std::lround对浮点数进行四舍五入取整运算
  2，利用boost的iostreams/filter/gzip对字符串压缩与解压缩
*/

#ifndef CARTOGRAPHER_COMMON_PORT_H_
#define CARTOGRAPHER_COMMON_PORT_H_

#include <cinttypes>
#include <cmath>
#include <string>

#include <boost/iostreams/device/back_inserter.hpp>
#include <boost/iostreams/filter/gzip.hpp>     //包含多种解压与压缩算法
#include <boost/iostreams/filtering_stream.hpp>//配合filter实现流过滤

using int8 = int8_t;
using int16 = int16_t;
using int32 = int32_t;
using int64 = int64_t;
using uint8 = uint8_t;
using uint16 = uint16_t;
using uint32 = uint32_t;
using uint64 = uint64_t;

using std::string;
namespace cartographer {
namespace common {

/*
lround(+2.3) = 2  lround(+2.5) = 3  lround(+2.7) = 3
lround(-2.3) = -2  lround(-2.5) = -3  lround(-2.7) = -3
lround(-0.0) = 0
*/
inline int RoundToInt(const float x) { return std::lround(x); }//四舍五入取整运算

inline int RoundToInt(const double x) { return std::lround(x); }

inline int64 RoundToInt64(const float x) { return std::lround(x); }

inline int64 RoundToInt64(const double x) { return std::lround(x); }

/*
利用gzip_compressor对string进行压缩，第一参数是未压缩string，第二参数是完成压缩string
*/
inline void FastGzipString(const string& uncompressed, string* compressed) {
  boost::iostreams::filtering_ostream out;//创建过滤流
  out.push(
      boost::iostreams::gzip_compressor(boost::iostreams::zlib::best_speed));//使用快速压缩算法
  out.push(boost::iostreams::back_inserter(*compressed));//对compressed 使用后插迭代器
  boost::iostreams::write(out,
                          reinterpret_cast<const char*>(uncompressed.data()),
                          uncompressed.size());//压缩 char *,插入compressed
}

/*
利用gzip_decompressor解压缩string，第一参数是待解压的string，第二参数是解压后的string
*/
inline void FastGunzipString(const string& compressed, string* decompressed) {
  boost::iostreams::filtering_ostream out;        //创建过滤流
  out.push(boost::iostreams::gzip_decompressor());//指定解压缩算法
  out.push(boost::iostreams::back_inserter(*decompressed));//对decompressed使用后插入迭代器
  boost::iostreams::write(out, reinterpret_cast<const char*>(compressed.data()),
                          compressed.size());//解压缩char*，插入decompressed
}

}  // namespace common
}  // namespace cartographer

#endif  // CARTOGRAPHER_COMMON_PORT_H_


/*

c++11 已经支持back_inserter。
std::back_inserter执行push_back操作, 返回值back_insert_iterator, 并实现自增.
具体用法可见 port_le_test.cpp
*/


/*
reinterpret_cast的转换格式：reinterpret_cast <type-id> (expression)
允许将任何指针类型转换为指定的指针类型
*/


/*
boost::iostreams/filtering_ostream用法：
 
压缩时
filtering_ostream out;
out.push(gzip_compressor()); //gzip OutputFilter
out.push(bzip2_compressor());//bzip2 OutputFilter
out.push(boost::iostreams::file_sink("test.txt"));//以file_sink device结束
将流的数据先按gzip压缩，然后再按bzip2压缩之后，才输出到text.txt文件中。

解压时
filtering_istream in;
in.push(gzip_decompressor());/gzip InputFilter
in.push(bzip2_decompressor());/bzip2 InputFilter
in.push(file_source("test.txt"));
先将test.txt文件中数据读出，然后按bzip2解压，然后再按gzip解压，存入in流中，正好是压缩的逆序。
*/
ref:
http://blog.csdn.net/fhxpp_27/article/details/8473852
```

本文发于：
*  http://www.jianshu.com/u/9e38d2febec1 
*  https://zhuanlan.zhihu.com/learnmoreonce
*  http://blog.csdn.net/learnmoreonce
*  slam源码分析微信公众号:slamcode