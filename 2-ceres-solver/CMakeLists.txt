
cmake_minimum_required( VERSION 2.8 )
project(test )

set( CMAKE_CXX_FLAGS "-std=c++11  -w  " ) 

find_package( Ceres REQUIRED )
#find_package( Gflags REQUIRED )

include_directories( ${CERES_INCLUDE_DIRS} )
include_directories(${GFLAGS_LIBRARIES})
add_definitions(-DCERES_GFLAGS_NAMESPACE=${GFLAGS_NAMESPACE})



file( GLOB APP_SOURCES "*.cc" )
foreach( testsourcefile ${APP_SOURCES} )   
    string( REPLACE ".cc" "" testname ${testsourcefile} )  
    get_filename_component(testname ${testname} NAME)
    
    add_executable( ${testname}  ${testsourcefile} )
    target_link_libraries( ${testname}  ${CERES_LIBRARIES}
     ${GFLAGS_LIBRARIES}
    )
    message( "add file: " ${testname} )

endforeach( testsourcefile ${APP_SOURCES} )


#----gtest-----
file( GLOB APP_SOURCES "*.cxx" )
foreach( testsourcefile ${APP_SOURCES} )   
    string( REPLACE ".cxx" "" testname ${testsourcefile} )  
    get_filename_component(testname ${testname} NAME)
    
    add_executable( ${testname}  ${testsourcefile} )
    target_link_libraries( ${testname}  ${CERES_LIBRARIES}
     ${GFLAGS_LIBRARIES}
      /usr/local/lib/libgtest_main.a

      /usr/local/lib/libgtest.a
      /usr/local/lib/libtest_util.a

      /usr/local/lib/libceres.a
      /usr/local/lib/libglog.a
      /usr/local/lib/libgmock.a

      /usr/local/lib/libgmock_main.a
    )
    message( "add file: " ${testname} )

endforeach( testsourcefile ${APP_SOURCES} )