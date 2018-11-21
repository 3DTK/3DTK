/*** Versions.cpp example: get versions of the E57 implementation library */
//! @file
//! @brief example: get versions of the E57 implementation library
#include <iostream>
#include "E57Foundation.h"
using namespace e57;
using namespace std;

//! @brief Example use of E57Utilities::getVersion function
int main(int /*argc*/, char** /*argv*/) {
    int astmMajor;
    int astmMinor;
    ustring libraryId;
    E57Utilities().getVersions(astmMajor, astmMinor, libraryId);

    cout << "astm major version     = " << astmMajor << endl;        
    cout << "astm minor version     = " << astmMinor << endl;        
    cout << "implementation library = " << libraryId << endl;        
    return(0);
}
