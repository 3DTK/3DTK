/*** SourceDestBufferFunctions.cpp example: get info about a SourceDestBuffer */
//! @file
//! @brief example: get info about a SourceDestBuffer
#include <iostream>
#include "E57Foundation.h"
using namespace e57;
using namespace std;

//! @brief Example use of SourceDestBuffer functions
int main(int /*argc*/, char** /*argv*/) {
    const int N = 4;

    try {
        ImageFile imf("temp._e57", "w");
        StructureNode root = imf.root();
        cout << "  imf.isWritable()=" << imf.isWritable() << endl << endl;
        
        StructureNode prototype(imf);
        prototype.set("cartesianX", FloatNode(imf));
        
        VectorNode codecs(imf, true);

        CompressedVectorNode points(imf, prototype, codecs);
        root.set("points", points);

        static double cartesianX[N] = {1.0, 2.0, 3.0, 4.0};
        vector<SourceDestBuffer> sbufs;
        sbufs.push_back(SourceDestBuffer(imf, "cartesianX",  cartesianX,  N));

        {
            cout << "  imf.writerCount()=" << imf.writerCount() << endl << endl;

            CompressedVectorWriter writer = points.writer(sbufs);

            cout << "  imf.writerCount()=" << imf.writerCount() << endl;
            cout << "  writer.isOpen()=" << writer.isOpen() << endl << endl;

            writer.write(N);

            cout << "  imf.writerCount()=" << imf.writerCount() << endl;
            cout << "  writer.isOpen()=" << writer.isOpen() << endl << endl;

            writer.close(); // don't forget to explicitly close the CompressedVectorWriter

            cout << "  imf.writerCount()=" << imf.writerCount() << endl;
            cout << "  writer.isOpen()=" << writer.isOpen() << endl;
        }
        cout << "  imf.writerCount()=" << imf.writerCount() << endl;

        cout << "pathName             = " << sbufs[0].pathName()             << endl;
        cout << "memoryRepresentation = " << sbufs[0].memoryRepresentation() << endl;
        cout << "capacity             = " << sbufs[0].capacity()             << endl;
        cout << "doConversion         = " << sbufs[0].doConversion()         << endl;
        cout << "doScaling            = " << sbufs[0].doScaling()            << endl;
        cout << "stride               = " << sbufs[0].stride()               << endl;

        imf.close(); // don't forget to explicitly close the ImageFile
        cout << "  imf.isWritable()=" << imf.isWritable() << endl;
    } catch(E57Exception& ex) {
        ex.report(__FILE__, __LINE__, __FUNCTION__);
        return(-1);
    }

    cout << "================" << endl;
    cout << "Opening file for reading" << endl;
    try {
        /// Read file from disk
        ImageFile imf("temp._e57", "r");
        StructureNode root = imf.root();
        cout << "  imf.isWritable()=" << imf.isWritable() << endl << endl;
        CompressedVectorNode points(root.get("/points"));

        double values[N];
        vector<SourceDestBuffer> dbufs;
        dbufs.push_back(SourceDestBuffer(imf, "cartesianX", values, N, true));
        
        unsigned gotCount = 0;
        {
            cout << "  imf.readerCount()=" << imf.readerCount() << endl << endl;

            CompressedVectorReader reader = points.reader(dbufs);

            cout << "  imf.readerCount()=" << imf.readerCount() << endl;
            cout << "  reader.isOpen()=" << reader.isOpen() << endl << endl;

            gotCount = reader.read();

            cout << "  imf.readerCount()=" << imf.readerCount() << endl;
            cout << "  reader.isOpen()=" << reader.isOpen() << endl << endl;

            reader.close(); // don't forget to explicitly close the CompressedVectorReader

            cout << "  imf.readerCount()=" << imf.readerCount() << endl;
            cout << "  reader.isOpen()=" << reader.isOpen() << endl;
        }
        cout << "  imf.readerCount()=" << imf.readerCount() << endl << endl;

        cout << "gotCount=" << gotCount << " records" << endl;

        for (unsigned i=0; i < gotCount; i++)
            cout << "value[" << i << "]=" << values[i] << endl;

        imf.close(); // don't forget to explicitly close the ImageFile
    } catch(E57Exception& ex) {
        ex.report(__FILE__, __LINE__, __FUNCTION__);
        return(-1);
    }
    return(0);
}
