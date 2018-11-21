/*** RawXML.cpp example: read XML section from E57 file */
//! @file
//! @brief example: read XML section from E57 file
#include <iostream>
#include "E57Foundation.h"
using namespace e57;
using namespace std;

//! @brief Example use of functions to extract XML section of E57 file.
int main(int /*argc*/, char** /*argv*/) {
    ustring inputFileName = "temp._e57";
    try {
        ImageFile imf(inputFileName, "w");
        StructureNode root = imf.root();
        
        root.set("greeting", StringNode(imf, "Hello world."));

        imf.close(); // don't forget to explicitly close the ImageFile
    } catch(E57Exception& ex) {
        ex.report(__FILE__, __LINE__, __FUNCTION__);
        return(-1);
    }

    try {
        E57Utilities utils = E57Utilities();

        static uint8_t buf[8];
        uint64_t length = utils.rawXmlLength(inputFileName);
        
        size_t count = 0;
        for (uint64_t start = 0; start < length; start += count) {
            if (length-start > sizeof(buf))
                count = sizeof(buf);
            else
                count = static_cast<size_t>(length-start);
 
            utils.rawXmlRead(inputFileName, buf, start, count);

            cout.write(reinterpret_cast<char*>(buf), static_cast<std::streamsize>(count));
        }
    } catch(E57Exception& ex) {
        ex.report(__FILE__, __LINE__, __FUNCTION__);
        return(-1);
    }
    return(0);
}
