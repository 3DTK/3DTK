//
// Created by Joschka van der Lucht on 02.03.18.
//

#ifndef INC_3DTK_CALIBRATOR_H
#define INC_3DTK_CALIBRATOR_H


namespace Calibrator{
    class Calibrator{
    private:

    public:
        void calibrateIntrinsic2D();
        void calibtrateIntrinsic3D();
        void calibtrateExtrinsic2D();
        void calibtrateExtrinsic3D();
    };
}


#endif //INC_3DTK_CALIBRATOR_H
