#include <iostream>
#include <fstream>
#include <string>

#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>


template<class T>
void saveSerializedObject(const T &s, const char * filename){
    // make an archive
    std::ofstream ofs(filename);
    boost::archive::text_oarchive oa(ofs);
    oa << s;
}


template<class T>
void restoreSerializedObject(T &s, const char * filename)
{
    // open the archive
    std::ifstream ifs(filename);
    boost::archive::text_iarchive ia(ifs);

    // restore the schedule from the archive
    ia >> s;
}
