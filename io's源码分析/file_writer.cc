
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
