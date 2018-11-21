/*** SourceDestBufferStringCreate.cpp example: creating SourceDestBuffers of Strings */
//! @file
//! @brief example: creating SourceDestBuffers of Strings
#include <iostream>
#include "E57Foundation.h"
using namespace e57;
using namespace std;

//! @brief Example use of ustring SourceDestBuffers.
int main(int /*argc*/, char** /*argv*/) {
    try {
        ImageFile imf("temp._e57", "w");
        StructureNode root = imf.root();
        
        StructureNode prototype(imf);
        prototype.set("s",  StringNode(imf));
        
        VectorNode codecs(imf, true);

        CompressedVectorNode cv(imf, prototype, codecs);
        root.set("cv1", cv);

        const int N = 2;
        vector<ustring> strings;
        strings.push_back("first string");
        strings.push_back("second string");
        vector<SourceDestBuffer> sbufs;
        sbufs.push_back(SourceDestBuffer(imf, "s", &strings));

        {
            CompressedVectorWriter writer = cv.writer(sbufs);
            writer.write(N);
            writer.close(); // don't forget to explicitly close the CompressedVectorWriter
        }

        imf.close(); // don't forget to explicitly close the ImageFile
    } catch(E57Exception& ex) {
        ex.report(__FILE__, __LINE__, __FUNCTION__);
        return(-1);
    }

    try {
        ImageFile imf("temp._e57", "r");
        StructureNode root = imf.root();

        CompressedVectorNode cv = static_cast<CompressedVectorNode> (root.get("cv1"));

        unsigned nChildren = static_cast<unsigned>(cv.childCount());
        vector<ustring> strings(nChildren);
        vector<SourceDestBuffer> dbufs;
        dbufs.push_back(SourceDestBuffer(imf, "s", &strings));

        {
            CompressedVectorReader reader = cv.reader(dbufs);
            unsigned nRead = reader.read();
            if (nRead != nChildren)
                return(-1);
            for (unsigned i = 0; i < nRead; i++)
                cout << "  string[" << i << "] = " << strings[i] << endl;
            reader.close(); // don't forget to explicitly close the CompressedVectorReader
        }

        imf.close(); // don't forget to explicitly close the ImageFile
    } catch(E57Exception& ex) {
        ex.report(__FILE__, __LINE__, __FUNCTION__);
        return(-1);
    }
    return(0);
}
