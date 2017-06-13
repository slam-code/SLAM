 #include <vector>
#include <iostream>

 /*
  */
	//#include<boost/iostreams/device/back_inserter.hpp>
	#include<vector>
	#include<iostream>
	using std::cout;
	using std::endl;
	int main(int argc, char *argv[])
	{
	    std::vector<int> vec;
	    for (int i = 0 ; i < 3; ++i)
		vec.push_back(i);

	    std::vector <int>::iterator vIter;
	    std::cout << "The initial vector vec is: ( ";
	    for (vIter = vec.begin(); vIter != vec.end(); vIter++)
		std::cout << *vIter << " ";
	    std::cout << ")." << std::endl;

	    // Insertions can be done with template function
	    std::back_insert_iterator<std::vector<int> > backiter(vec);
	    *backiter = 30;
	    backiter++;
	    *backiter = 40;

	    // Alternatively, insertions can be done with the
	    // back_insert_iterator member function
	    std::back_inserter(vec) = 500;//从vector后插入
	    std::back_inserter(vec) = 600;

	    std::cout << "After the insertions, the vector vec is: ( ";
	    for (vIter = vec.begin(); vIter != vec.end(); vIter++)
		std::cout << *vIter << " ";
	    std::cout << ")." << std::endl<<"----\n\n";


	std::vector<int> firstvector, secondvector;  
    for (int i=1; i<=5; i++)  
    {   
        firstvector.push_back(i);   
        secondvector.push_back(i*10);  
    }  
      
    // 在firstvector向量后面加入secondvector的值.  
    std::copy(secondvector.begin(), secondvector.end(), std::back_inserter(firstvector));  
      
    std::vector<int>::iterator it;  
    for ( it = firstvector.begin(); it!= firstvector.end(); ++it )  
        std::cout << *it << " ";  
    std::cout << std::endl;

	    return 0;
	}