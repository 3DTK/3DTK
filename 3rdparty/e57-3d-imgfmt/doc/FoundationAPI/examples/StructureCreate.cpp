/*** StructureCreate.cpp example: creating StructureNodes */
//! @file
//! @brief example: creating StructureNodes
#include <iostream>
#include "E57Foundation.h"
using namespace e57;
using namespace std;

//! @brief Example use of StructureNode functions
int main(int /*argc*/, char** /*argv*/) {
    try {
        ImageFile imf("temp._e57", "w");
        StructureNode root = imf.root();

        // Add three elements: /child1, /child1/grandchild, /child2
        StructureNode child1  = StructureNode(imf);
        StructureNode child2  = StructureNode(imf);
        StringNode grandchild = StringNode(imf, "I'm a grandchild");
        root.set("child1", child1);
        root.set("child2", child2);
        child1.set("grandchild", grandchild);

        cout << "root has " << root.childCount() << " children:" << endl;
        for (int i = 0; i < root.childCount(); i++) {
            Node n = root.get(i);
            if (n.type() == E57_STRUCTURE) {
                StructureNode s = static_cast<StructureNode>(n);
                cout << "  " << s.pathName() << " Structure with "
                     << s.childCount() << " child elements" << endl;
           } else
                cout << "  " << n.pathName() << endl;
        }

        if (child1.isDefined("grandchild"))
            cout << "/child1 has grandchild element" << endl;

        if (!child1.isDefined("/child2/grandchild"))
            cout << "/child2 has no grandchild element" << endl;

        ustring absolutePathName = "/child1/grandchild";
        if (child1.isDefined(absolutePathName)) {
            Node n = root.get(absolutePathName);
            cout << n.pathName() << " exists" << endl;
        }

        imf.close(); // don't forget to explicitly close the ImageFile
    } catch(E57Exception& ex) {
        ex.report(__FILE__, __LINE__, __FUNCTION__);
        return(-1);
    }
    return(0);
}

