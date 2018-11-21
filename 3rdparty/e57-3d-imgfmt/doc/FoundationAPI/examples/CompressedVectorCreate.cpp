/*** CompressedVectorCreate.cpp example: creating CompressedVectorNodes from arrays */
//! @file
//! @brief example: creating CompressedVectorNodes from arrays
#include <iostream>
#include "E57Foundation.h"
using namespace e57;
using namespace std;

//! @cond documentNonPublic   The following isn't part of the API, and isn't documented.
struct XYZStruct {
    double x;   //!< x ordinate
    double y;   //!< y ordinate
    double z;   //!< z ordinate
};
//! @endcond

//! @brief Example use of CompressedVectorNode creation and write/read
int main(int /*argc*/, char** /*argv*/) {
    try {
        ImageFile imf("temp._e57", "w");
        StructureNode root = imf.root();
        
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
        sourceBuffers.push_back(SourceDestBuffer(imf, "/cartesianX",  cartesianX,  N));
        sourceBuffers.push_back(SourceDestBuffer(imf, "/cartesianY",  cartesianY,  N));
        sourceBuffers.push_back(SourceDestBuffer(imf, "/cartesianZ",  cartesianZ,  N));

        {
            CompressedVectorWriter writer = cv.writer(sourceBuffers);
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

        CompressedVectorNode cv = static_cast<CompressedVectorNode> (root.get("points"));

        const int N = 4;
        XYZStruct cartesianXYZ[N];
        vector<SourceDestBuffer> destBuffers;
        destBuffers.push_back(SourceDestBuffer(imf, "/cartesianX", &cartesianXYZ[0].x, N,
                                               false, false, sizeof(XYZStruct)));
        destBuffers.push_back(SourceDestBuffer(imf, "/cartesianY", &cartesianXYZ[0].y, N,
                                               false, false, sizeof(XYZStruct)));
        destBuffers.push_back(SourceDestBuffer(imf, "/cartesianZ", &cartesianXYZ[0].z, N,
                                               false, false, sizeof(XYZStruct)));

        cout << "CompressedVector has " << cv.childCount() << " child elements" << endl;
        {
            CompressedVectorReader reader = cv.reader(destBuffers);
            
            uint64_t totalRead = 0;
            unsigned nRead = 0;
            while ((nRead = reader.read(destBuffers)) > 0) {
                cout << "Got " << nRead << " points" << endl;
                for (unsigned i = 0; i < nRead; i++) {
                    cout << "point " << totalRead + i << endl;
                    cout << "  cartesianX = " << cartesianXYZ[i].x << endl;
                    cout << "  cartesianY = " << cartesianXYZ[i].y << endl;
                    cout << "  cartesianZ = " << cartesianXYZ[i].z << endl;
                }
                totalRead += nRead;
            }
            reader.close(); // don't forget to explicitly close the CompressedVectorReader
        }

        imf.close(); // don't forget to explicitly close the ImageFile
    } catch(E57Exception& ex) {
        ex.report(__FILE__, __LINE__, __FUNCTION__);
        return(-1);
    }
    return(0);
}
