/*** IntegerCreate.cpp example: creating IntegerNodes */
//! @file
//! @brief example: creating IntegerNodes
#include <iostream>
#include "E57Foundation.h"
using namespace e57;
using namespace std;

//! @brief Get and print some info about an IntegerNode
void printIntegerInfo(ImageFile imf, ustring pathName)
{
    cout << pathName << ":" << endl;

    StructureNode root = imf.root();

    if (root.isDefined(pathName)) {
        Node n = root.get(pathName);
        if (n.type() == E57_INTEGER) {
            IntegerNode iNode = static_cast<IntegerNode>(n);
            cout << "  value   = " << iNode.value()   << endl;
            cout << "  minimum = " << iNode.minimum() << endl;
            cout << "  maximum = " << iNode.maximum() << endl;
        } else
            cout << "oops " << n.pathName() << " isn't an Integer" << endl;
    }
}

//! @brief Example use of IntegerNode functions
int main(int /*argc*/, char** /*argv*/) {
    try {
        ImageFile imf("temp._e57", "w");
        StructureNode root = imf.root();

        // Create 4 example Integers 
        root.set("i1", IntegerNode(imf));
        root.set("i2", IntegerNode(imf, 123));
        root.set("i3", IntegerNode(imf, 123, 0, 1023));
        root.set("i4", IntegerNode(imf, 0, -128, 127));

        printIntegerInfo(imf, "/i1");
        printIntegerInfo(imf, "/i2");
        printIntegerInfo(imf, "/i3");
        printIntegerInfo(imf, "/i4");

        imf.close(); // don't forget to explicitly close the ImageFile
    } catch(E57Exception& ex) {
        ex.report(__FILE__, __LINE__, __FUNCTION__);
        return(-1);
    }
    return(0);
}
