/*
 * motion3-horn implementation
 *
 * Copyright (C) Stanislav Serebryakov
 *
 * Released under the GPL version 3.
 *
 */

#include <cv.h>

static inline CvMat *mat3D() { return cvCreateMat(3,1,CV_32FC1); }

static CvMat *matFromP3D(CvPoint3D32f p) {
    CvMat *m = mat3D(); 
    CV_MAT_ELEM(*m, float, 0, 0) = p.x;
    CV_MAT_ELEM(*m, float, 1, 0) = p.y;
    CV_MAT_ELEM(*m, float, 2, 0) = p.z;
    return m;
}


// union vectors from point correspondaces (as in Horn method)
static void constructXYZ( CvMat *p1, CvMat *p2, CvMat *p3
                 , CvMat *x, CvMat *y, CvMat *z) {
    CvMat *p2Mp1 = mat3D();
    cvSub(p2, p1, p2Mp1, NULL);
    cvConvertScale(p2Mp1, x, 1.0/cvNorm(p2Mp1, NULL, CV_L2, NULL), 0);

    CvMat *p3Mp1 = mat3D();
    cvSub(p3, p1, p3Mp1, NULL);
    CvMat *ySndItem = mat3D();
    cvConvertScale(x, ySndItem, cvDotProduct(p3Mp1, x), NULL);
    CvMat *ySub = mat3D();
    cvSub(p3Mp1, ySndItem, ySub);
    cvConvertScale(ySub, y, 1.0/cvNorm(ySub, NULL, CV_L2, NULL), 0);

    cvCrossProduct(x, y, z);

    cvReleaseMat(&p2Mp1);
    cvReleaseMat(&p3Mp1);
    cvReleaseMat(&ySndItem);
    cvReleaseMat(&ySub);
}

static void fillM(CvMat *m, CvMat *x, CvMat *y, CvMat *z) {
    CV_MAT_ELEM(*m, float, 0, 0) = CV_MAT_ELEM(*x, float, 0, 0);
    CV_MAT_ELEM(*m, float, 1, 0) = CV_MAT_ELEM(*x, float, 1, 0);
    CV_MAT_ELEM(*m, float, 2, 0) = CV_MAT_ELEM(*x, float, 2, 0);
    CV_MAT_ELEM(*m, float, 0, 1) = CV_MAT_ELEM(*y, float, 0, 0);
    CV_MAT_ELEM(*m, float, 1, 1) = CV_MAT_ELEM(*y, float, 1, 0);
    CV_MAT_ELEM(*m, float, 2, 1) = CV_MAT_ELEM(*y, float, 2, 0);
    CV_MAT_ELEM(*m, float, 0, 2) = CV_MAT_ELEM(*z, float, 0, 0);
    CV_MAT_ELEM(*m, float, 1, 2) = CV_MAT_ELEM(*z, float, 1, 0);
    CV_MAT_ELEM(*m, float, 2, 2) = CV_MAT_ELEM(*z, float, 2, 0);
} 


// Horn's fast method
void estimatePose3D(CvPoint3D32f *prev, CvPoint3D32f *curr, CvMat *rot, CvMat *trn) {
    CvMat *x1 = mat3D();
    CvMat *y1 = mat3D(); 
    CvMat *z1 = mat3D();
    CvMat *x2 = mat3D();
    CvMat *y2 = mat3D(); 
    CvMat *z2 = mat3D();
    CvMat *p1 = matFromP3D(prev[0]);
    CvMat *p2 = matFromP3D(prev[1]);
    CvMat *p3 = matFromP3D(prev[2]);
    CvMat *c1 = matFromP3D(curr[0]);
    CvMat *c2 = matFromP3D(curr[1]);
    CvMat *c3 = matFromP3D(curr[2]);

    constructXYZ(p1, p2, p3, x1, y1, z1);
    constructXYZ(c1, c2, c3, x2, y2, z2);
    CvMat *m1 = cvCreateMat(3, 3, CV_32FC1);
    CvMat *m2 = cvCreateMat(3, 3, CV_32FC1);
    fillM(m2, x2, y2, z2);
    fillM(m1, x1, y1, z1);
    CvMat *rotM = cvCreateMat(3, 3, CV_32FC1);
    cvGEMM(m1, m2, 1.0, NULL, 1.0, rotM, CV_GEMM_B_T);

    CvMat *rp1 = mat3D(); 
    cvGEMM(rotM, p1, 1.0, NULL, 1.0, rp1, 0);
    CvMat *rp2 = mat3D(); 
    cvGEMM(rotM, p2, 1.0, NULL, 1.0, rp2, 0);
    CvMat *rp3 = mat3D(); 
    cvGEMM(rotM, p3, 1.0, NULL, 1.0, rp3, 0);
    CvMat *rp12 = mat3D();
    cvAdd(rp1, rp2, rp12, NULL);
    CvMat *rp123 = mat3D();
    cvAdd(rp12, rp3, rp123, NULL);
    CvMat *c12 = mat3D();
    cvAdd(c1, c2, c12, NULL);
    CvMat *c123 = mat3D();
    cvAdd(c12, c3, c123, NULL);
    CvMat *cg1 = mat3D();
    CvMat *cg2 = mat3D();
    cvConvertScale(c123, cg2, 1.0/3.0, 0.0);
    cvConvertScale(rp123, cg1, 1.0/3.0, 0.0);
    cvSub(cg2, cg1, trn, NULL);
    cvRodrigues2(rotM, rot, NULL);

    cvReleaseMat(&x1);
    cvReleaseMat(&y1);
    cvReleaseMat(&z1);
    cvReleaseMat(&x2);
    cvReleaseMat(&y2);
    cvReleaseMat(&z2);
    cvReleaseMat(&p1);
    cvReleaseMat(&p2);
    cvReleaseMat(&p3);
    cvReleaseMat(&c1);
    cvReleaseMat(&c2);
    cvReleaseMat(&c3);
    cvReleaseMat(&m1);
    cvReleaseMat(&m2);
    cvReleaseMat(&rotM);
    cvReleaseMat(&rp1);
    cvReleaseMat(&rp2);
    cvReleaseMat(&rp3);
    cvReleaseMat(&rp12);
    cvReleaseMat(&rp123);
    cvReleaseMat(&c12);
    cvReleaseMat(&c12);
    cvReleaseMat(&cg1);
    cvReleaseMat(&cg2);
    //FIXME: mb missed smthg :P
}
