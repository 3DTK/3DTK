/*** HelloWorld.cpp example: the basics */
//! @file
//! @brief example: the basics
#include <iostream>
#include "E57Foundation.h"
using namespace e57;
using namespace std;

//! @brief Example use of ImageFile
int main(int /*argc*/, char** /*argv*/) {
    try {
        ImageFile imf("temp._e57", "w");
        StructureNode root = imf.root();
        
        root.set("greeting", StringNode(imf, "Hello world."));

        imf.close(); // don't forget to explicitly close the ImageFile
    } catch(E57Exception& ex) {
        ex.report(__FILE__, __LINE__, __FUNCTION__);
        return(-1);
    }

    try {
        ImageFile imf("temp._e57", "r");
        StructureNode root = imf.root();

        StringNode greeting(root.get("greeting"));
        cout << "Value of /greeting = " << greeting.value() << endl;

        imf.close(); // don't forget to explicitly close the ImageFile
    } catch(E57Exception& ex) {
        ex.report(__FILE__, __LINE__, __FUNCTION__);
        return(-1);
    }
    return(0);
}
