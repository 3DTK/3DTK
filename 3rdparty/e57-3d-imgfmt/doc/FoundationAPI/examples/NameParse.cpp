/*** NameParse.cpp example: parse good and bad element names */
//! @file
//! @brief example: parse good and bad element names
#include <iostream>
#include "E57Foundation.h"
using namespace e57;
using namespace std;

//! @brief Try various parsing functions on an element name.
void tryParse(ImageFile imf, ustring pathName) {
    cout << "pathName=" << pathName << endl;

    try {
        ustring prefix, localPart;
        imf.elementNameParse(pathName, prefix, localPart);
        cout << "  prefix=\"" << prefix << "\" localPart=\"" << localPart << "\"" << endl;
    } catch(E57Exception& ex) {
        if (ex.errorCode() == E57_ERROR_BAD_PATH_NAME)
            cout << "  Is not legal element name" << endl;
    }

    if (imf.isElementNameExtended(pathName))
        cout << "  Is extended element name" << endl;
    else
        cout << "  Is not extended element name" << endl;
}

//! @brief Example use of element name parsing functions
int main(int /*argc*/, char** /*argv*/) {
    try {
        ImageFile imf("temp._e57", "w");

        tryParse(imf, "bar");
        tryParse(imf, "foo:bar");

        imf.close(); // don't forget to explicitly close the ImageFile
    } catch(E57Exception& ex) {
        ex.report(__FILE__, __LINE__, __FUNCTION__);
        return(-1);
    }
    return(0);
}
