/*** ScaledIntegerCreate.cpp example: creating ScaledIntegerNodes */
//! @file
//! @brief example: creating ScaledIntegerNodes
#include <iostream>
#include "E57Foundation.h"
using namespace e57;
using namespace std;

//! @brief Get and print some info about a ScaledIntegerNode
void printScaledIntegerInfo(ImageFile imf, ustring pathName)
{
    cout << pathName << ":" << endl;

    StructureNode root = imf.root();

    if (root.isDefined(pathName)) {
        Node n = root.get(pathName);
        if (n.type() == E57_SCALED_INTEGER) {
            ScaledIntegerNode si = static_cast<ScaledIntegerNode>(n);
            cout << "  rawValue    = " << si.rawValue()      << endl;
            cout << "  scaledValue = " << si.scaledValue()   << endl;
            cout << "  minimum     = " << si.minimum()    << endl;
            cout << "  maximum     = " << si.maximum()    << endl;
            cout << "  scale       = " << si.scale()      << endl;
            cout << "  offset      = " << si.offset()     << endl;
        } else
            cout << "oops " << n.pathName() << " isn't an ScaledInteger" << endl;
    }
}

//! @brief Example use of ScaledIntegerNode functions
int main(int /*argc*/, char** /*argv*/) {
    try {
        ImageFile imf("temp._e57", "w");
        StructureNode root = imf.root();

        // Create 5 example ScaledIntegers
        root.set("si1", ScaledIntegerNode(imf, int64_t(), E57_INT64_MIN, E57_INT64_MAX));
        root.set("si2", ScaledIntegerNode(imf, int64_t(123), E57_INT64_MIN, E57_INT64_MAX));
        root.set("si3", ScaledIntegerNode(imf, 123, 0, 1023));
        root.set("si4", ScaledIntegerNode(imf, 123, 0, 1023, .001));
        root.set("si5", ScaledIntegerNode(imf, 123, 0, 1023, .001, 100.0));

        printScaledIntegerInfo(imf, "/si1");
        printScaledIntegerInfo(imf, "/si2");
        printScaledIntegerInfo(imf, "/si3");
        printScaledIntegerInfo(imf, "/si4");
        printScaledIntegerInfo(imf, "/si5");

        imf.close(); // don't forget to explicitly close the ImageFile
    } catch(E57Exception& ex) {
        ex.report(__FILE__, __LINE__, __FUNCTION__);
        return(-1);
    }
    return(0);
}
