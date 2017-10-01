
源码可在https://github.com/learnmoreonce/SLAM 下载


``` c++
 
文件:io/file_writer.h


#ifndef CARTOGRAPHER_IO_FILE_WRITER_H_
#define CARTOGRAPHER_IO_FILE_WRITER_H_

#include <fstream>
#include <functional>
#include <memory>

#include "cartographer/common/port.h"

namespace cartographer {
namespace io {
/*
FileWriter负责文件写入的虚基类,不可拷贝/赋值
没有数据成员.只提供一系列抽象接口.
1),WriteHeader(),写入文件head
2),Write(),写入数据
3),Close(),关闭文件
*/
// Simple abstraction for a file.
class FileWriter {
 public:
  FileWriter() {}
  FileWriter(const FileWriter&) = delete;
  FileWriter& operator=(const FileWriter&) = delete;

  virtual ~FileWriter() {}

  // Write 'data' to the beginning of the file. This is required to overwrite
  // fixed size headers which contain the number of points once we actually know
  // how many points there are.
  virtual bool WriteHeader(const char* data, size_t len) = 0;

  virtual bool Write(const char* data, size_t len) = 0;
  virtual bool Close() = 0;
};

/*
StreamFileWriter 文件流写入类,继承自FileWriter
数据成员只有一个std::ofstream out_负责将文件写入磁盘

*/
// An Implementation of file using std::ofstream.
class StreamFileWriter : public FileWriter {
 public:
  ~StreamFileWriter() override;

  StreamFileWriter(const string& filename);

  bool Write(const char* data, size_t len) override;
  bool WriteHeader(const char* data, size_t len) override; //从文件开始处写入,覆盖旧数据
  bool Close() override;

 private:
  std::ofstream out_;
};

/*工厂模式,
创建一个FileWriter对象,由智能指针管理生命期,
返回值是std::unique_ptr<FileWriter>,
函数参数是string.
*/
using FileWriterFactory =
    std::function<std::unique_ptr<FileWriter>(const string& filename)>;

}  // namespace io
}  // namespace cartographer

#endif  // CARTOGRAPHER_IO_FILE_WRITER_H_
```

```c++

io/file_writer.cc


#include "cartographer/io/file_writer.h"

namespace cartographer {
namespace io {

//out:File open for writing: the internal stream buffer supports output operations.
//binary:Operations are performed in binary mode rather than text.

StreamFileWriter::StreamFileWriter(const string& filename)
    : out_(filename, std::ios::out | std::ios::binary) {}//打开,并以2进制方式写入

StreamFileWriter::~StreamFileWriter() {}

//写入len个char
bool StreamFileWriter::Write(const char* const data, const size_t len) {
  if (out_.bad()) {
    return false;
  }
  out_.write(data, len);
  return !out_.bad();
}

bool StreamFileWriter::Close() { //关闭文件
  if (out_.bad()) {
    return false;
  }
  out_.close();
  return !out_.bad();
}

bool StreamFileWriter::WriteHeader(const char* const data, const size_t len) {
  if (out_.bad()) {
    return false;
  }
  out_.flush();  //清空buffer
  out_.seekp(0); //偏移量为0
  return Write(data, len);
}

}  // namespace io
}  // namespace cartographer


```

本文发于：
*  http://www.jianshu.com/u/9e38d2febec1
*  https://zhuanlan.zhihu.com/learnmoreonce
*  http://blog.csdn.net/learnmoreonce
*  slam源码分析微信公众号:slamcode
