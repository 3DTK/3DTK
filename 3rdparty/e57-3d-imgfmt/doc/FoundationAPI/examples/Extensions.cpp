/*** Extensions.cpp example: register use of extended element names */
//! @file
//! @brief example: register use of extended element names
#include <iostream>
#include "E57Foundation.h"
using namespace e57;
using namespace std;

//! @brief Example use of E57 extension functions
int main(int /*argc*/, char** /*argv*/) {
    try {
        ImageFile imf("temp._e57", "w");

        imf.extensionsAdd("ext1", "http://www.example.com/MyExtension1");

        ustring uri;
        if (!imf.extensionsLookupPrefix("ext1", uri))
            cout << "Oops, ext1 should have been defined" << endl;
        else
            cout << "ext1 is prefix for uri=" << uri << endl;

        ustring prefix;
        if (!imf.extensionsLookupUri("http://www.example.com/MyExtension1", prefix))
            cout << "Oops, URI http://www.example.com/MyExtension1 should have been defined" << endl;
        else
            cout << "URI http://www.example.com/MyExtension1 has prefix=" << prefix << endl;

        cout << "Defined extensions:" << endl;
        for (int i = 0; i < imf.extensionsCount(); i++)
            cout << "  prefix:" << imf.extensionsPrefix(i) << " URI:" << imf.extensionsUri(i) << endl;

        imf.root().set("ext1:greeting", StringNode(imf, "Hello world."));

        imf.close(); // don't forget to explicitly close the ImageFile
    } catch(E57Exception& ex) {
        ex.report(__FILE__, __LINE__, __FUNCTION__);
        return(-1);
    }
    return(0);
}

