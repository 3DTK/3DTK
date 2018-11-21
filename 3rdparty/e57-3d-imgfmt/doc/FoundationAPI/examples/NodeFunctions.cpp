/*** NodeFunctions.cpp example: get info about generic Nodes */
//! @file
//! @brief example: get info about generic Nodes
#include <iostream>
#include "E57Foundation.h"
using namespace e57;
using namespace std;

//! @brief Print generic Node info.
void printGenericInfo(Node n)
{
    cout << "  elementName    = " << n.elementName()              << endl;
    cout << "  pathName       = " << n.pathName()                 << endl;
    cout << "  parent name    = " << n.parent().pathName()        << endl;
    cout << "  isRoot         = " << n.isRoot()                   << endl;
    cout << "  isAttached     = " << n.isAttached()               << endl;
    cout << "  destImageFile  = " << n.destImageFile().fileName() << endl;
    cout << "  type           = " << n.type()                     << endl;
}

//! @brief Decode Node type and print type specific info.
void printSpecificInfo(Node n)
{
    switch (n.type()) {
        case E57_STRUCTURE: {
            StructureNode s = static_cast<StructureNode>(n);
            StructureNode s2(n);
            StructureNode s3 = StructureNode(n);
            StructureNode s4 = (StructureNode)n;
            cout << "  StructureNode with " << s.childCount() << " children" << endl;
            printGenericInfo(s);
        }
        break;
        case E57_VECTOR: {
            VectorNode v = static_cast<VectorNode>(n);
            cout << "  VectorNode with " << v.childCount() << " children" << endl;
            printGenericInfo(v);
        }
        break;
        case E57_COMPRESSED_VECTOR: {
            CompressedVectorNode cv = static_cast<CompressedVectorNode>(n);
            cout << "  CompressedVectorNode with " << cv.childCount() << " children" << endl;
            printGenericInfo(cv);
        }
        break;
        case E57_INTEGER: {
            IntegerNode i = static_cast<IntegerNode>(n);
            cout << "  IntegerNode with value = " << i.value() << endl;
            printGenericInfo(i);
        }
        break;
        case E57_SCALED_INTEGER: {
            ScaledIntegerNode si = static_cast<ScaledIntegerNode>(n);
            cout << "  ScaledIntegerNode with rawValue = " << si.rawValue() << endl;
            printGenericInfo(si);
        }
        break;
        case E57_FLOAT: {
            FloatNode f = static_cast<FloatNode>(n);
            cout << "  FloatNode with value = " << f.value() << endl;
            printGenericInfo(f);
        }
        break;
        case E57_STRING: {
            StringNode s = static_cast<StringNode>(n);
            cout << "  StringNode with value = " << s.value() << endl;
            printGenericInfo(s);
        }
        break;
        case E57_BLOB: {
            BlobNode b = static_cast<BlobNode>(n);
            cout << "  BlobNode with length = " << b.byteCount() << endl;
            printGenericInfo(b);
        }
        break;
    }
}

//! @brief Example use of generic and specific Node functions.
int main(int /*argc*/, char** /*argv*/) {
    try {
        ImageFile imf("temp._e57", "w");
        StructureNode root = imf.root();

        // Add a String element in /child/grandchild
        StructureNode child   = StructureNode(imf);
        StringNode grandchild = StringNode(imf, "I'm a grandchild");
        root.set("child", child);
        child.set("grandchild", grandchild);

        cout << "root node:" << endl;
        printSpecificInfo(root);

        cout << "child node:" << endl;
        printSpecificInfo(child);

        cout << "grandchild node:" << endl;
        printSpecificInfo(grandchild);

        StructureNode unattached = StructureNode(imf);
        StringNode unattachedChild = StringNode(imf, "I'm a child of an unattached node");
        unattached.set("unattachedChild", unattachedChild);

        cout << "unattached node:" << endl;
        printSpecificInfo(unattached);

        cout << "unattached child node:" << endl;
        printSpecificInfo(unattachedChild);

        imf.close(); // don't forget to explicitly close the ImageFile
    } catch(E57Exception& ex) {
        ex.report(__FILE__, __LINE__, __FUNCTION__);
        return(-1);
    }
    return(0);
}
