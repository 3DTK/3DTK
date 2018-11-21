/*** VectorFunctions.cpp example: get info about VectorNodes */
//! @file
//! @brief example: get info about VectorNodes
#include <iostream>
#include "E57Foundation.h"
using namespace e57;
using namespace std;

//! @brief Example use of VectorNode functions
int main(int /*argc*/, char** /*argv*/) {
    try {
        ImageFile imf("temp._e57", "w");
        StructureNode root = imf.root();

        // Create vector /exampleVector, with 2 child elements
        VectorNode v(imf, false);
        root.set("exampleVector", v);
        v.append(IntegerNode(imf, 100));
        v.append(IntegerNode(imf, 101));

        cout << v.pathName() << " has " << v.childCount() << " children:" << endl;
        for (int i = 0; i < v.childCount(); i++) {
            Node n = v.get(i);
            if (n.type() == E57_INTEGER) {
                IntegerNode iNode = static_cast<IntegerNode>(n);
                cout << "  " << iNode.pathName() << " IntegerNode = " << iNode.value() << endl;
           } else
                cout << "  " << n.pathName() << endl;
        }

        if (v.isDefined("1"))
            cout << v.pathName() << " has a second child element" << endl;
        if (!v.isDefined("2"))
            cout << v.pathName() << " doesn't have a third child element" << endl;

        Node n = root.get("/exampleVector");
        if (n.type() == E57_VECTOR) {
            VectorNode v2 = static_cast<VectorNode>(n);
            cout << v2.pathName() << " has " << v2.childCount() << " children" << endl;
        }

        imf.close(); // don't forget to explicitly close the ImageFile
    } catch(E57Exception& ex) {
        ex.report(__FILE__, __LINE__, __FUNCTION__);
        return(-1);
    }
    return(0);
}
