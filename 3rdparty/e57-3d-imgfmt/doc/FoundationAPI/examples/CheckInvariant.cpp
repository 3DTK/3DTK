/*** CheckInvariant.cpp example: use checkInvariant function for ImageFile and other objects */
//! @file
//! @brief example: use checkInvariant function for ImageFile and other objects.
#include <iostream>
#include "E57Foundation.h"
using namespace e57;
using namespace std;

//! @brief Example use of various checkInvariant functions
int main(int /*argc*/, char** /*argv*/) {
    try {
        ImageFile imf("temp._e57", "w");
            imf.checkInvariant();
        StructureNode root = imf.root();
            root.checkInvariant();
            imf.checkInvariant();
            

        // Add three elements: /child1, /child1/grandchild, /child2
        StructureNode child1  = StructureNode(imf);
            child1.checkInvariant();
            imf.checkInvariant();
        StructureNode child2  = StructureNode(imf);
            child2.checkInvariant();
            imf.checkInvariant();
        StringNode grandchild = StringNode(imf, "I'm a grandchild");
            grandchild.checkInvariant();
            imf.checkInvariant();
        root.set("child1", child1);
            root.checkInvariant();
            child1.checkInvariant();
        root.set("child2", child2);
            root.checkInvariant();
            child2.checkInvariant();
        child1.set("grandchild", grandchild);
            child1.checkInvariant();
            grandchild.checkInvariant();

        Node n = root.get("child2");
            n.checkInvariant();

        // Create vector /exampleVector, with 2 child elements
        VectorNode v(imf, false);
            v.checkInvariant();
            imf.checkInvariant();
        root.set("exampleVector", v);
            root.checkInvariant();
            v.checkInvariant();
        v.append(IntegerNode(imf, 100));
            v.checkInvariant();
            imf.checkInvariant();
        v.append(IntegerNode(imf, 101));
            v.checkInvariant();
            imf.checkInvariant();

        IntegerNode iNode(imf, 100);
            iNode.checkInvariant();
            imf.checkInvariant();
        root.set("exampleInteger", iNode);
            root.checkInvariant();
            iNode.checkInvariant();

        ScaledIntegerNode si(imf, 123, 0, 1023, .001, 100.0);
            si.checkInvariant();
            imf.checkInvariant();
        root.set("si1", si);
            root.checkInvariant();
            si.checkInvariant();

        FloatNode f(imf, 123.45F);
            f.checkInvariant();
            imf.checkInvariant();
        root.set("f1", f);
            root.checkInvariant();
            f.checkInvariant();

        StringNode s(imf, "a random string");
            s.checkInvariant();
            imf.checkInvariant();
        root.set("s1", s);
            root.checkInvariant();
            s.checkInvariant();

        const int blobLength = 10;
        static uint8_t buf[blobLength] = {10, 9, 8, 7, 6, 5, 4, 3, 2, 1};
        BlobNode b = BlobNode(imf, blobLength);
            b.checkInvariant();
            imf.checkInvariant();
        root.set("blob1", b);
            root.checkInvariant();
            b.checkInvariant();
        b.write(buf, 0, blobLength);
            b.checkInvariant();

        StructureNode prototype(imf);
            prototype.checkInvariant();
            imf.checkInvariant();
        prototype.set("cartesianX", FloatNode(imf));
            prototype.checkInvariant();
            imf.checkInvariant();
        prototype.set("cartesianY", FloatNode(imf));
            prototype.checkInvariant();
            imf.checkInvariant();
        prototype.set("cartesianZ", FloatNode(imf));
            prototype.checkInvariant();
            imf.checkInvariant();
        
        VectorNode codecs(imf, true);
            codecs.checkInvariant();
            imf.checkInvariant();

        CompressedVectorNode cv(imf, prototype, codecs);
            cv.checkInvariant();
            imf.checkInvariant();
            prototype.checkInvariant();
            codecs.checkInvariant();
        root.set("points", cv);
            root.checkInvariant();
            cv.checkInvariant();

        const int N = 4;
        static double cartesianX[N] = {1.0, 2.0, 3.0, 4.0};
        static double cartesianY[N] = {1.1, 2.1, 3.1, 4.1};
        static double cartesianZ[N] = {1.2, 2.2, 3.2, 4.2};
        vector<SourceDestBuffer> sourceBuffers;
        sourceBuffers.push_back(SourceDestBuffer(imf, "cartesianX",  cartesianX,  N));
            sourceBuffers[0].checkInvariant();
            imf.checkInvariant();
        sourceBuffers.push_back(SourceDestBuffer(imf, "cartesianY",  cartesianY,  N));
            sourceBuffers[1].checkInvariant();
            imf.checkInvariant();
        sourceBuffers.push_back(SourceDestBuffer(imf, "cartesianZ",  cartesianZ,  N));
            sourceBuffers[2].checkInvariant();
            imf.checkInvariant();
        {
            CompressedVectorWriter writer = cv.writer(sourceBuffers);
                writer.checkInvariant();
                sourceBuffers[0].checkInvariant();
                sourceBuffers[1].checkInvariant();
                sourceBuffers[2].checkInvariant();
            writer.write(N);
                writer.checkInvariant();
        }

        imf.extensionsAdd("ext1", "http://www.example.com/MyExtension1");
            imf.checkInvariant();
        root.set("ext1:greeting", StringNode(imf, "Hello world."));
            root.checkInvariant();
            imf.checkInvariant();

        imf.close(); // don't forget to explicitly close the ImageFile
            imf.checkInvariant();
    } catch(E57Exception& ex) {
        ex.report(__FILE__, __LINE__, __FUNCTION__);
        return(-1);
    }

    try {
        ImageFile imf("temp._e57", "r");
            imf.checkInvariant();
        StructureNode root = imf.root();
            root.checkInvariant();
            imf.checkInvariant();

        CompressedVectorNode cv = static_cast<CompressedVectorNode> (root.get("points"));
            cv.checkInvariant();
            root.checkInvariant();

        const int N = 10;
        static double cartesianX[N];
        static double cartesianY[N];
        static double cartesianZ[N];
        vector<SourceDestBuffer> destBuffers;
        destBuffers.push_back(SourceDestBuffer(imf, "cartesianX", cartesianX, N));
            destBuffers[0].checkInvariant();
            imf.checkInvariant();
        destBuffers.push_back(SourceDestBuffer(imf, "cartesianY", cartesianY, N));
            destBuffers[1].checkInvariant();
            imf.checkInvariant();
        destBuffers.push_back(SourceDestBuffer(imf, "cartesianZ", cartesianZ, N));
            destBuffers[2].checkInvariant();
            imf.checkInvariant();

        if (cv.prototype().type() != E57_STRUCTURE)
            return(-1);
            cv.checkInvariant();
        StructureNode prototype = static_cast<StructureNode>(cv.prototype());
            prototype.checkInvariant();
            cv.checkInvariant();
        {
            CompressedVectorReader reader = cv.reader(destBuffers);
                reader.checkInvariant();
                cv.checkInvariant();
            
            uint64_t totalRead = 0;
            unsigned nRead = 0;
            while ((nRead = reader.read(destBuffers)) > 0) {
                reader.checkInvariant();
                destBuffers[0].checkInvariant();
                destBuffers[1].checkInvariant();
                destBuffers[2].checkInvariant();

                cout << "Got " << nRead << " points" << endl;
                for (unsigned i = 0; i < nRead; i++) {
                    cout << "point " << totalRead + i << endl;
                    cout << "  cartesianX = " << cartesianX[i] << endl;
                    cout << "  cartesianY = " << cartesianY[i] << endl;
                    cout << "  cartesianZ = " << cartesianZ[i] << endl;
                }
                totalRead += nRead;
            }
            reader.checkInvariant();
            destBuffers[0].checkInvariant();
            destBuffers[1].checkInvariant();
            destBuffers[2].checkInvariant();

            reader.close(); // don't forget to explicitly close the CompressedVectorReader
                reader.checkInvariant();
        }
        imf.checkInvariant();

        imf.close(); // don't forget to explicitly close the ImageFile
            imf.checkInvariant();

        cout << "***** Got to end without any checkInvariant exceptions" << endl;
    } catch(E57Exception& ex) {
        ex.report(__FILE__, __LINE__, __FUNCTION__);
        return(-1);
    }
    return(0);
}
