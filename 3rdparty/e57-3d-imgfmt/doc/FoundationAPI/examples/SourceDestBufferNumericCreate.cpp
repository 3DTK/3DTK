/*** SourceDestBufferIntegerCreate.cpp example: creating SourceDestBuffers of numbers */
//! @file
//! @brief example: creating SourceDestBuffers of numbers
#include <iostream>
#include "E57Foundation.h"
using namespace e57;
using namespace std;

//! @brief Example use of numeric SourceDestBuffers.
int main(int /*argc*/, char** /*argv*/) {
    const unsigned N = 4;

    try {
        ImageFile imf("temp._e57", "w");
        StructureNode root = imf.root();
        
        StructureNode prototype(imf);
        prototype.set("i8",  ScaledIntegerNode(imf, 0, -10000, 10000, 0.001, 0.0));
        prototype.set("u8",  ScaledIntegerNode(imf, 0, -10000, 10000, 0.001, 0.0));
        prototype.set("i16", ScaledIntegerNode(imf, 0, -10000, 10000, 0.001, 0.0));
        prototype.set("u16", ScaledIntegerNode(imf, 0, -10000, 10000, 0.001, 0.0));
        prototype.set("i32", ScaledIntegerNode(imf, 0, -10000, 10000, 0.001, 0.0));
        prototype.set("u32", ScaledIntegerNode(imf, 0, -10000, 10000, 0.001, 0.0));
        prototype.set("i64", ScaledIntegerNode(imf, 0, -10000, 10000, 0.001, 0.0));
        prototype.set("i64Scaled", ScaledIntegerNode(imf, 0, -10000, 10000, 0.001, 0.0));
        prototype.set("f", FloatNode(imf, 0.0, E57_SINGLE));
        prototype.set("d", FloatNode(imf, 0.0, E57_DOUBLE));
        
        VectorNode codecs(imf, true);

        CompressedVectorNode cv(imf, prototype, codecs);
        root.set("cv1", cv);

        static int8_t   i8[N]  = {1,2,3,4};
        static uint8_t  u8[N]  = {2,3,4,5};
        static int16_t  i16[N] = {3,4,5,6};
        static uint16_t u16[N] = {4,5,6,7};
        static int32_t  i32[N] = {5,6,7,8};
        static uint32_t u32[N] = {6,7,8,9};
        static int64_t  i64[N] = {7,8,9,10};
        static float    f[N]   = {8.1F,9.1F,10.1F,11.1F};
        static double   d[N]   = {9.1,10.1,11.1,12.1};
        vector<SourceDestBuffer> sbufs;
        sbufs.push_back(SourceDestBuffer(imf, "i8",  i8,  N));
        sbufs.push_back(SourceDestBuffer(imf, "u8",  u8,  N));
        sbufs.push_back(SourceDestBuffer(imf, "i16", i16, N));
        sbufs.push_back(SourceDestBuffer(imf, "u16", u16, N));
        sbufs.push_back(SourceDestBuffer(imf, "i32", i32, N));
        sbufs.push_back(SourceDestBuffer(imf, "u32", u32, N));
        sbufs.push_back(SourceDestBuffer(imf, "i64", i64, N));
        sbufs.push_back(SourceDestBuffer(imf, "i64Scaled", i64, N, false, true));
        sbufs.push_back(SourceDestBuffer(imf, "f", f, N));
        sbufs.push_back(SourceDestBuffer(imf, "d", d, N));

        {
            CompressedVectorWriter writer = cv.writer(sbufs);
            cout << "Writing " << N << " records to " << writer.compressedVectorNode().pathName() << endl;
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

        cout << cv.pathName() << " has " << cv.childCount() << " child elements" << endl;

        if (cv.prototype().type() != E57_STRUCTURE)
            return(-1);
        StructureNode prototype = static_cast<StructureNode>(cv.prototype());
        cout << "The prototype has " << prototype.childCount() << " child elements" << endl;

        VectorNode codecs(cv.codecs());
        cout << "The codecs has " << codecs.childCount() << " child elements" << endl;

        for (int i = 0; i < prototype.childCount(); i++) {
            Node n = prototype.get(i);

            cout << "reading " << n.pathName() << ":" << endl;
            try {
                static double rawValue[N];
                vector<SourceDestBuffer> dbufs1;
                dbufs1.push_back(SourceDestBuffer(imf, n.pathName(), rawValue, N, true, false));

                CompressedVectorReader reader = cv.reader(dbufs1);
                if (reader.read() != N)
                    return(-1);
                cout << "  read " << N << " records from " << reader.compressedVectorNode().pathName() << endl;

                for (unsigned j = 0; j < N; j++)
                    cout << "  rawValue[" << j << "]    = " << rawValue[j] << endl;
                reader.close(); // don't forget to explicitly close the CompressedVectorReader
            } catch(E57Exception& ex) {
                ex.report(__FILE__, __LINE__, __FUNCTION__);
                cout << "**** Reading " << n.pathName() << " failed" << endl;
            }

            try {
                static double scaledValue[N];
                vector<SourceDestBuffer> dbufs2;
                dbufs2.push_back(SourceDestBuffer(imf, n.pathName(), scaledValue, N, true, true));

                CompressedVectorReader reader = cv.reader(dbufs2);
                if (reader.read() != N)
                    return(-1);
                for (unsigned j = 0; j < N; j++)
                    cout << "  scaledValue[" << j << "] = " << scaledValue[j] << endl;
                reader.close(); // don't forget to explicitly close the CompressedVectorReader
            } catch(E57Exception& /*ex*/) {
                cout << "**** Reading " << n.pathName() << " failed" << endl;
            }
        }

        imf.close(); // don't forget to explicitly close the ImageFile
    } catch(E57Exception& ex) {
        ex.report(__FILE__, __LINE__, __FUNCTION__);
        return(-1);
    }
    return(0);
}
