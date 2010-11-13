#include <cxtypes.h>

// Direct 3<->3 points estimation. rot and trn is out parameters (should be initialized before).
void estimatePose3D(CvPoint3D32f *prev, CvPoint3D32f *curr, CvMat *rot, CvMat *trn);

