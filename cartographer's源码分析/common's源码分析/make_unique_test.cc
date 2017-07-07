
#include <iostream>
#include <string>
#include "cartographer/common/make_unique.h"
#include "gtest/gtest.h"
//using namespace std;
using std::cout;
using std::cin;
using std::endl;
using std::string;
namespace cartographer {
namespace common {
TEST(MAKE_UNIQUE ,make_unique) {
    cout << *make_unique<int>() << endl;
    cout << *make_unique<int>(1729) << endl;
    cout << "\"" << *make_unique<string>() << "\"" << endl;
    cout << "\"" << *make_unique<string>("meow") << "\"" << endl;
    cout << "\"" << *make_unique<string>(6, 'z') << "\"" << endl;

    auto up = make_unique<int[]>(5);

    for (int i = 0; i < 5; ++i) {
        cout << up[i] << " ";
    }

    cout << endl;

    #if defined(ERROR1)
        auto up1 = make_unique<string[]>("error");
    #elif defined(ERROR2)
        auto up2 = make_unique<int[]>(10, 20, 30, 40);
    #elif defined(ERROR3)
        auto up3 = make_unique<int[5]>();
    #elif defined(ERROR4)
        auto up4 = make_unique<int[5]>(11, 22, 33, 44, 55);
    #endif  
}

/*
输出:

0
1729
""
"meow"
"zzzzzz"
0 0 0 0 0


*/
}
}