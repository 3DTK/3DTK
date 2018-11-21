/*** BlobCreate.cpp example: creating BlobNodes */
//! @file
//! @brief example: creating BlobNodes
#include <iostream>
#include "E57Foundation.h"
using namespace e57;
using namespace std;

//! @brief Example use of BlobNode functions
int main(int /*argc*/, char** /*argv*/) {
    try {
        ImageFile imf("temp._e57", "w");
        StructureNode root = imf.root();
        
        const int blobLength = 10;
        static uint8_t buf[blobLength] = {10, 9, 8, 7, 6, 5, 4, 3, 2, 1};

        BlobNode b = BlobNode(imf, blobLength);
        root.set("blob1", b);

        const int halfLength = blobLength/2;
        b.write(buf, 0, halfLength);
        b.write(&buf[halfLength], halfLength, halfLength);

        imf.close(); // don't forget to explicitly close the ImageFile
    } catch(E57Exception& ex) {
        ex.report(__FILE__, __LINE__, __FUNCTION__);
        return(-1);
    }

    try {
        ImageFile imf("temp._e57", "r");
        StructureNode root = imf.root();

        BlobNode b = static_cast<BlobNode>(root.get("/blob1"));

        cout << "Blob length is " << b.byteCount() << " bytes" << endl;
        for (int64_t i = 0; i < b.byteCount(); i++) {
            static uint8_t buf[1];
            b.read(buf, i, 1);
            cout << "  byte " << i << " = " << static_cast<unsigned>(buf[0]) << endl;
        }

        imf.close(); // don't forget to explicitly close the ImageFile
    } catch(E57Exception& ex) {
        ex.report(__FILE__, __LINE__, __FUNCTION__);
        return(-1);
    }
    return(0);
}
