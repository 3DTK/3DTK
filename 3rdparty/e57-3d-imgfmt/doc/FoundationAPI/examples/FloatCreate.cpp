/*** FloatCreate.cpp example: creating FloatNodes */
//! @file
//! @brief example: creating FloatNodes
#include <iostream>
#include "E57Foundation.h"
using namespace e57;
using namespace std;

//! @brief Get and print some info about a FloatNode
void printFloatInfo(ImageFile imf, ustring pathName)
{
    cout << pathName << ":" << endl;

    StructureNode root = imf.root();

    if (root.isDefined(pathName)) {
        Node n = root.get(pathName);
        if (n.type() == E57_FLOAT) {
            FloatNode f = static_cast<FloatNode>(n);
            cout << "  value     = " << f.value()     << endl;
            cout << "  precision = " << f.precision() << endl;
            cout << "  minimum   = " << f.minimum()   << endl;
            cout << "  maximum   = " << f.maximum()   << endl;
        } else
            cout << "oops " << n.pathName() << " isn't an Float" << endl;
    }
}

//! @brief Example use of FloatNode functions
int main(int /*argc*/, char** /*argv*/) {
    try {
        ImageFile imf("temp._e57", "w");
        StructureNode root = imf.root();

        // Create 7 example Floats 
        root.set("f1", FloatNode(imf));
        root.set("f2", FloatNode(imf, 123.45F));
        root.set("f3", FloatNode(imf, 123.45));
        root.set("f4", FloatNode(imf, 123.45F, E57_SINGLE));
        root.set("f5", FloatNode(imf, 123.45,  E57_SINGLE));
        root.set("f6", FloatNode(imf, 123.45,  E57_DOUBLE));
        root.set("f7", FloatNode(imf, 123.45,  E57_DOUBLE, 0.0, 1023.0));

        printFloatInfo(imf, "/f1");
        printFloatInfo(imf, "/f2");
        printFloatInfo(imf, "/f3");
        printFloatInfo(imf, "/f4");
        printFloatInfo(imf, "/f5");
        printFloatInfo(imf, "/f6");
        printFloatInfo(imf, "/f7");

        imf.close(); // don't forget to explicitly close the ImageFile
    } catch(E57Exception& ex) {
        ex.report(__FILE__, __LINE__, __FUNCTION__);
        return(-1);
    }
    return(0);
}
