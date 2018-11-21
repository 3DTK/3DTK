/*** ImageFileDump.cpp example: print diagnostics about an ImageFile and other objects */
//! @file
//! @brief example: print diagnostics about an ImageFile and other objects
#include <iostream>
#include "E57Foundation.h"
using namespace e57;
using namespace std;

//! @brief Example use of various dump functions
int main(int /*argc*/, char** /*argv*/) {
    try {
        ImageFile imf("temp._e57", "w");
        StructureNode root = imf.root();

        // Add three elements: /child1, /child1/grandchild, /child2
        cout << "================" << endl;
        StructureNode child1  = StructureNode(imf);
        StructureNode child2  = StructureNode(imf);
        StringNode grandchild = StringNode(imf, "I'm a grandchild");
        root.set("child1", child1);
        root.set("child2", child2);
        child1.set("grandchild", grandchild);
        cout << "root node:" << endl;
        root.dump(4);

        cout << "================" << endl;
        Node n = root.get("child2");
        cout << n.pathName() << " generic Node:" << endl;
        n.dump(4);

        // Create vector /exampleVector, with 2 child elements
        cout << "================" << endl;
        VectorNode v(imf, false);
        root.set("exampleVector", v);
        v.append(IntegerNode(imf, 100));
        v.append(IntegerNode(imf, 101));
        cout << v.pathName() << " VectorNode:" << endl;
        v.dump(4);

        cout << "================" << endl;
        IntegerNode iNode(imf, 100);
        root.set("exampleInteger", iNode);
        cout << iNode.pathName() << " IntegerNode:" << endl;
        iNode.dump(4);

        cout << "================" << endl;
        ScaledIntegerNode si(imf, 123, 0, 1023, .001, 100.0);
        root.set("si1", si);
        cout << si.pathName() << " ScaledIntegerNode:" << endl;
        si.dump(4);

        cout << "================" << endl;
        FloatNode f(imf, 123.45F);
        root.set("f1", f);
        cout << f.pathName() << " FloatNode:" << endl;
        f.dump(4);

        cout << "================" << endl;
        StringNode s(imf, "a random string");
        root.set("s1", s);
        cout << s.pathName() << " StringNode:" << endl;
        s.dump(4);

        cout << "================" << endl;
        const int blobLength = 10;
        static uint8_t buf[blobLength] = {10, 9, 8, 7, 6, 5, 4, 3, 2, 1};
        BlobNode b = BlobNode(imf, blobLength);
        root.set("blob1", b);
        b.write(buf, 0, blobLength);
        cout << b.pathName() << " BlobNode:" << endl;
        b.dump(4);

        cout << "================" << endl;
        StructureNode prototype(imf);
        prototype.set("cartesianX", FloatNode(imf));
        prototype.set("cartesianY", FloatNode(imf));
        prototype.set("cartesianZ", FloatNode(imf));
        
        VectorNode codecs(imf, true);

        CompressedVectorNode cv(imf, prototype, codecs);
        root.set("points", cv);

        const int N = 4;
        static double cartesianX[N] = {1.0, 2.0, 3.0, 4.0};
        static double cartesianY[N] = {1.1, 2.1, 3.1, 4.1};
        static double cartesianZ[N] = {1.2, 2.2, 3.2, 4.2};
        vector<SourceDestBuffer> sourceBuffers;
        sourceBuffers.push_back(SourceDestBuffer(imf, "cartesianX",  cartesianX,  N));
        sourceBuffers.push_back(SourceDestBuffer(imf, "cartesianY",  cartesianY,  N));
        sourceBuffers.push_back(SourceDestBuffer(imf, "cartesianZ",  cartesianZ,  N));

        {
            CompressedVectorWriter writer = cv.writer(sourceBuffers);
            writer.write(N);

            cout << "CompressedVectorWriter:" << endl;
            writer.dump(4);
        }

        cout << "================" << endl;
        cout << cv.pathName() << " CompressedVectorNode:" << endl;
        cv.dump(4);

        cout << "================" << endl;
        imf.extensionsAdd("ext1", "http://www.example.com/MyExtension1");
        root.set("ext1:greeting", StringNode(imf, "Hello world."));
        cout << imf.fileName() << " ImageFile:" << endl;
        imf.dump(4);

        imf.close(); // don't forget to explicitly close the ImageFile
    } catch(E57Exception& ex) {
        ex.report(__FILE__, __LINE__, __FUNCTION__);
        return(-1);
    }

    try {
        ImageFile imf("temp._e57", "r");
        StructureNode root = imf.root();

        CompressedVectorNode cv = static_cast<CompressedVectorNode> (root.get("points"));

        const int N = 2;
        static double cartesianX[N];
        static double cartesianY[N];
        static double cartesianZ[N];
        vector<SourceDestBuffer> destBuffers;
        destBuffers.push_back(SourceDestBuffer(imf, "cartesianX", cartesianX, N));
        destBuffers.push_back(SourceDestBuffer(imf, "cartesianY", cartesianY, N));
        destBuffers.push_back(SourceDestBuffer(imf, "cartesianZ", cartesianZ, N));

        cout << cv.pathName() << " has " << cv.childCount() << " child elements" << endl;
        if (cv.prototype().type() != E57_STRUCTURE)
            return(-1);
        StructureNode prototype = static_cast<StructureNode>(cv.prototype());
        cout << "The prototype has " << prototype.childCount() << " child elements" << endl;
        {
            CompressedVectorReader reader = cv.reader(destBuffers);
            
            uint64_t totalRead = 0;
            unsigned nRead = 0;
            while ((nRead = reader.read(destBuffers)) > 0) {
                cout << "Got " << nRead << " points" << endl;
                for (unsigned i = 0; i < nRead; i++) {
                    cout << "point " << totalRead + i << endl;
                    cout << "  cartesianX = " << cartesianX[i] << endl;
                    cout << "  cartesianY = " << cartesianY[i] << endl;
                    cout << "  cartesianZ = " << cartesianZ[i] << endl;
                }
                totalRead += nRead;

            }
            cout << "================" << endl;
            cout << "CompressedVectorReader:" << endl;
            reader.dump(4);

            reader.close(); // don't forget to explicitly close the CompressedVectorReader
        }

        imf.close(); // don't forget to explicitly close the ImageFile
    } catch(E57Exception& ex) {
        ex.report(__FILE__, __LINE__, __FUNCTION__);
        return(-1);
    }
    return(0);
}
