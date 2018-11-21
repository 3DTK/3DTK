/*** Cancel.cpp example: stop writing after discover an error */
//! @file
//! @brief example: stop writing after discover an error
#include <iostream>
#include "E57Foundation.h"
using namespace e57;
using namespace std;

//! @brief Print whether ImageFile is open
void testOpen(ImageFile imf)
{
    if (imf.isOpen())
        cout << "ImageFile is open" << endl;
    else
        cout << "ImageFile is closed" << endl;
}

//! @brief Example use of ImageFile::cancel
int main(int /*argc*/, char** /*argv*/) {
    try {
        ImageFile imf("temp._e57", "w");

        testOpen(imf);
        if (/*need to stop writing file*/ 1) {
            cout << "Canceling write to " << imf.fileName() << endl;
            imf.cancel();
            testOpen(imf);
            return(-1);
        }
           
        imf.close(); // don't forget to explicitly close the ImageFile
    } catch(E57Exception& ex) {
        ex.report(__FILE__, __LINE__, __FUNCTION__);
        return(-1);
    }
    return(0);
}
