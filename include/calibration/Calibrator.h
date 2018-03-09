//
// Created by Joschka van der Lucht on 02.03.18.
//

#ifndef INC_3DTK_CALIBRATOR_H
#define INC_3DTK_CALIBRATOR_H


namespace calibration{
    class Calibrator{
    private:

    public:
        void calibrateIntrinsic2D();
        void calibrateIntrinsic3D();
        void calibrateExtrinsic2D();
        void calibrateExtrinsic3D();
    };
}


#endif //INC_3DTK_CALIBRATOR_H
