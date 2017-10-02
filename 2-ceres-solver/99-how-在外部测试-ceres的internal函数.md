修改test_util.cc：

std::string TestFileAbsolutePath(const std::string& filename) {
  std::string FLAGS_test_srcdir="./";
 
  return JoinPath(FLAGS_test_srcdir + CERES_TEST_SRCDIR_SUFFIX,
                  filename);
}
