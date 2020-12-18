//
// Created by Joschka van der Lucht on 02.03.18.
//

#ifndef INC_3DTK_CALIBRATOR_H
#define INC_3DTK_CALIBRATOR_H

#include "AprilTag.h"

namespace calibration {
    class Calibrator {
    private:

    public:
        void calibrateIntrinsic();

        void calibrateExtrinsic();
    };
}


#endif //INC_3DTK_CALIBRATOR_H
