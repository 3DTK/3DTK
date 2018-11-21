/*** VectorCreate.cpp example: creating VectorNodes */
//! @file
//! @brief example: creating VectorNodes
#include <iostream>
#include "E57Foundation.h"
using namespace e57;
using namespace std;

//! @brief Example use of VectorNode creation
int main(int /*argc*/, char** /*argv*/) {
    try {
        ImageFile imf("temp._e57", "w");
        StructureNode root = imf.root();

        // Create vector /v1, with 2 child elements: /v1/0, /v1/1
        VectorNode v1(imf, false);
        root.set("v1", v1);
        v1.append(StringNode(imf, "some string"));
        v1.append(StringNode(imf, "another string"));

        // Create vector /v2, with 2 child elements: /v2/0, /v2/1
        VectorNode v2(imf, true);
        root.set("v2", v2);
        v2.append(StringNode(imf, "yet another string"));
        v2.append(FloatNode(imf, 1.234));
        
        if (!v1.allowHeteroChildren())
            cout << "/v1 cannot contain children with different types" << endl;
        if (v2.allowHeteroChildren())
            cout << "/v2 can contain children with different types" << endl;

        imf.close(); // don't forget to explicitly close the ImageFile
    } catch(E57Exception& ex) {
        ex.report(__FILE__, __LINE__, __FUNCTION__);
        return(-1);
    }
    return(0);
}
