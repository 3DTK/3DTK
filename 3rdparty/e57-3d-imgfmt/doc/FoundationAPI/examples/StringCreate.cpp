/*** StringCreate.cpp example: creating StringNodes */
//! @file
//! @brief example: creating StringNodes
#include <iostream>
#include "E57Foundation.h"
using namespace e57;
using namespace std;

//! @brief Get and print some info about a StringNode
void printStringInfo(ImageFile imf, ustring pathName)
{
    cout << pathName << ":" << endl;

    StructureNode root = imf.root();

    if (root.isDefined(pathName)) {
        Node n = root.get(pathName);
        if (n.type() == E57_STRING) {
            StringNode s = static_cast<StringNode>(n);
            cout << "  value = " << s.value() << endl;
        } else
            cout << "oops " << n.pathName() << " isn't an String" << endl;
    }
}

//! @brief Example use of StringNode functions
int main(int /*argc*/, char** /*argv*/) {
    try {
        ImageFile imf("temp._e57", "w");
        StructureNode root = imf.root();

        // Create 3 example Strings
        root.set("s1", StringNode(imf));
        root.set("s2", StringNode(imf, "hey there"));
        root.set("s3", StringNode(imf, "has a ]]> in it"));

        printStringInfo(imf, "/s1");
        printStringInfo(imf, "/s2");
        printStringInfo(imf, "/s3");

        imf.close(); // don't forget to explicitly close the ImageFile
    } catch(E57Exception& ex) {
        ex.report(__FILE__, __LINE__, __FUNCTION__);
        return(-1);
    }
    return(0);
}
