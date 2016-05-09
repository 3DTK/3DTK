//
// Created by Joschka van der Lucht on 07.12.15.
//

#ifndef CAMERACALIBRATIONTOOL_3DPATTERN_H
#define CAMERACALIBRATIONTOOL_3DPATTERN_H

#include <stdio.h>
#include <opencv2/core/core.hpp>

using namespace std;
using namespace cv;

//namespace Pattern3DApril {
    static vector<pair<int, Point3f * >> pattern;
    static Point3f point0 [4] = {Point3f(0, 0, 0)},
        point1 [4]= {Point3f(84.2496, 0, 0)},
        point2 [4]= {Point3f(0, 85.2151, 0)},
        point3 [4]= {Point3f(163.8720, -4.0778, -0.0722)},
        point4 [4]= {Point3f(208.0940, -3.9643, -0.0551)},
        point5 [4]= {Point3f(164.1060, 38.4906, -0.1688)},
        point6 [4]= {Point3f(80.8755, 80.8685, -0.1644)},
        point7 [4]= {Point3f(124.3620, 81.1722, -0.1680)},
        point8 [4]= {Point3f(80.0263, 124.4170, 0.0621)},
        point9 [4]= {Point3f(-3.9423, 167.1760, 0.6285)},
        point10 [4]= {Point3f(40.2084, 167.0310, 0.5332)},
        point11 [4]= {Point3f(-3.9075, 210.2870, 1.2585)},

        point12 [4]= {Point3f(784.4620, -1.4210, 5.3221)},
        point13 [4]= {Point3f(869.4630, -1.4409, 4.6849)},
        point14 [4]= {Point3f(869.3650, 83.7895, 4.2352)},
        point15 [4]= {Point3f(693.4390, -5.5525, 7.4285)},
        point16 [4]= {Point3f(737.5090, -5.7252, 6.3851)},
        point17 [4]= {Point3f(737.6210, 38.2438, 6.0012)},
        point18 [4]= {Point3f(776.4650, 80.3945, 5.0029)},
        point19 [4]= {Point3f(820.5300, 80.6308, 4.5774)},
        point20 [4]= {Point3f(820.5850, 124.4850, 4.2558)},
        point21 [4]= {Point3f(861.0130, 164.8200, 3.9875)},
        point22 [4]= {Point3f(904.3920, 164.9910, 4.0008)},
        point23 [4]= {Point3f(904.4640, 209.1880, 3.9590)},

        point24 [4]= {Point3f(-2.3070, 778.6930, -2.4653)},
        point25 [4]= {Point3f(-2.2117, 863.8110, -3.5506)},
        point26 [4]= {Point3f(82.1463, 863.7360, -1.7693)},
        point27 [4]= {Point3f(-6.6155, 687.2160, -0.7360)},
        point28 [4]= {Point3f(-6.4890, 730.1890, -1.6786)},
        point29 [4]= {Point3f(37.6100, 730.1100, -0.7028)},
        point30 [4]= {Point3f(77.7325, 772.0920, -0.5923)},
        point31 [4]= {Point3f(77.5719, 814.4130, -1.1379)},
        point32 [4]= {Point3f(121.5680, 814.3830, -0.6159)},
        point33 [4]= {Point3f(162.0010, 857.1860, 0.1377)},
        point34 [4]= {Point3f(162.0370, 899.5220, -0.0868)},
        point35 [4]= {Point3f(206.3570, 899.4890, 1.0588)},

        point36 [4]= {Point3f(866.4500, 782.1100, 1.7401)},
        point37 [4]= {Point3f(781.0320, 865.7440, 1.7728)},
        point38 [4]= {Point3f(864.9330, 866.9310, 0.9510)},
        point39 [4]= {Point3f(904.3440, 691.6280, 2.7538)},
        point40 [4]= {Point3f(859.2510, 734.1550, 2.8089)},
        point41 [4]= {Point3f(903.1970, 734.9920, 2.4500)},
        point42 [4]= {Point3f(820.4860, 777.0940, 2.5041)},
        point43 [4]= {Point3f(775.8930, 819.8040, 2.4915)},
        point44 [4]= {Point3f(819.8300, 820.0350, 1.9078)},
        point45 [4]= {Point3f(733.2420, 857.8980, 2.7095)},
        point46 [4]= {Point3f(688.7920, 900.1480, 2.6272)},
        point47 [4]= {Point3f(732.8240, 901.1880, 1.8827)},

        point48 [4]= {Point3f(466.7750, 268.0060, 201.0320)},
        point49 [4]= {Point3f(538.8220, 341.3570, 201.4290)},
        point50 [4]= {Point3f(611.1320, 415.4000, 201.7270)},
        point51 [4]= {Point3f(393.1290, 340.0280, 200.2210)},
        point52 [4]= {Point3f(465.1680, 413.7080, 200.2250)},
        point53 [4]= {Point3f(537.2070, 487.4020, 200.7670)},
        point54 [4]= {Point3f(319.3200, 412.5070, 199.9830)},
        point55 [4]= {Point3f(391.6430, 485.9800, 199.9940)},
        point56 [4]= {Point3f(463.7250, 559.7750, 200.6050)},

        point57 [4]= {Point3f(152.1990, 166.2550, 202.5280)},
        point58 [4]= {Point3f(178.7890, 193.0730, 202.5020)},
        point59 [4]= {Point3f(205.0080, 219.7880, 202.5530)},
        point60 [4]= {Point3f(232.1620, 247.0180, 202.6870)},
        point61 [4]= {Point3f(258.8440, 274.3030, 202.8410)},
        point62 [4]= {Point3f(285.6380, 301.1700, 202.9900)},
        point63 [4]= {Point3f(312.2500, 328.6460, 203.1280)},

        point64 [4]= {Point3f(759.2540, 153.2320, 206.4260)},
        point65 [4]= {Point3f(732.1970, 180.0640, 206.1400)},
        point66 [4]= {Point3f(705.1600, 206.7010, 205.8840)},
        point67 [4]= {Point3f(678.0890, 233.2230, 205.6860)},
        point68 [4]= {Point3f(651.4020, 259.839, 205.4800)},
        point69 [4]= {Point3f(624.0970, 286.6090, 205.2540)},
        point70 [4]= {Point3f(597.2040, 313.4290, 205.0810)},

        point71 [4]= {Point3f(588.0520, 606.1800, 204.5180)},
        point72 [4]= {Point3f(615.0810, 633.2880, 204.6100)},
        point73 [4]= {Point3f(641.7160, 659.9920, 204.6720)},
        point74 [4]= {Point3f(668.7430, 686.6390, 204.7750)},
        point75 [4]= {Point3f(695.5070, 713.7380, 204.8020)},
        point76 [4]= {Point3f(722.2890, 740.9740, 204.8880)},
        point77 [4]= {Point3f(749.2150, 767.7820, 205.0100)},

        point78 [4]= {Point3f(321.7180, 585.3140, 203.4150)},
        point79 [4]= {Point3f(294.7410, 612.2480, 203.1860)},
        point80 [4]= {Point3f(267.5750, 638.7290, 203.1860)},
        point81 [4]= {Point3f(240.0600, 665.2840, 203.0600)},
        point82 [4]= {Point3f(212.7340, 692.0150, 202.9390)},
        point83 [4]= {Point3f(185.9740, 718.7460, 202.6070)},
        point84 [4]= {Point3f(158.6690, 745.4550, 202.4340)};

    static void initPattern3Dapril() {
        pattern.push_back(pair<int, Point3f*>(0, point0));
        pattern.push_back(pair<int, Point3f*>(1, point1));
        pattern.push_back(pair<int, Point3f*>(2, point2));
        pattern.push_back(pair<int, Point3f*>(3, point3));
        pattern.push_back(pair<int, Point3f*>(4, point4));
        pattern.push_back(pair<int, Point3f*>(5, point5));
        pattern.push_back(pair<int, Point3f*>(6, point6));
        pattern.push_back(pair<int, Point3f*>(7, point7));
        pattern.push_back(pair<int, Point3f*>(8, point8));
        pattern.push_back(pair<int, Point3f*>(9, point9));
        pattern.push_back(pair<int, Point3f*>(10, point10));
        pattern.push_back(pair<int, Point3f*>(11, point11));
        pattern.push_back(pair<int, Point3f*>(12, point12));
        pattern.push_back(pair<int, Point3f*>(13, point13));
        pattern.push_back(pair<int, Point3f*>(14, point14));
        pattern.push_back(pair<int, Point3f*>(15, point15));
        pattern.push_back(pair<int, Point3f*>(16, point16));
        pattern.push_back(pair<int, Point3f*>(17, point17));
        pattern.push_back(pair<int, Point3f*>(18, point18));
        pattern.push_back(pair<int, Point3f*>(19, point19));
        pattern.push_back(pair<int, Point3f*>(20, point20));
        pattern.push_back(pair<int, Point3f*>(21, point21));
        pattern.push_back(pair<int, Point3f*>(22, point22));
        pattern.push_back(pair<int, Point3f*>(23, point23));
        pattern.push_back(pair<int, Point3f*>(24, point24));
        pattern.push_back(pair<int, Point3f*>(25, point25));
        pattern.push_back(pair<int, Point3f*>(26, point26));
        pattern.push_back(pair<int, Point3f*>(27, point27));
        pattern.push_back(pair<int, Point3f*>(28, point28));
        pattern.push_back(pair<int, Point3f*>(29, point29));
        pattern.push_back(pair<int, Point3f*>(30, point30));
        pattern.push_back(pair<int, Point3f*>(31, point31));
        pattern.push_back(pair<int, Point3f*>(32, point32));
        pattern.push_back(pair<int, Point3f*>(33, point33));
        pattern.push_back(pair<int, Point3f*>(34, point34));
        pattern.push_back(pair<int, Point3f*>(35, point35));
        pattern.push_back(pair<int, Point3f*>(36, point36));
        pattern.push_back(pair<int, Point3f*>(37, point37));
        pattern.push_back(pair<int, Point3f*>(38, point38));
        pattern.push_back(pair<int, Point3f*>(39, point39));
        pattern.push_back(pair<int, Point3f*>(40, point40));
        pattern.push_back(pair<int, Point3f*>(41, point41));
        pattern.push_back(pair<int, Point3f*>(42, point42));
        pattern.push_back(pair<int, Point3f*>(43, point43));
        pattern.push_back(pair<int, Point3f*>(44, point44));
        pattern.push_back(pair<int, Point3f*>(45, point45));
        pattern.push_back(pair<int, Point3f*>(46, point46));
        pattern.push_back(pair<int, Point3f*>(47, point47));
        pattern.push_back(pair<int, Point3f*>(48, point48));
        pattern.push_back(pair<int, Point3f*>(49, point49));
        pattern.push_back(pair<int, Point3f*>(50, point50));
        pattern.push_back(pair<int, Point3f*>(51, point51));
        pattern.push_back(pair<int, Point3f*>(52, point52));
        pattern.push_back(pair<int, Point3f*>(53, point53));
        pattern.push_back(pair<int, Point3f*>(54, point54));
        pattern.push_back(pair<int, Point3f*>(55, point55));
        pattern.push_back(pair<int, Point3f*>(56, point56));
        pattern.push_back(pair<int, Point3f*>(57, point57));
        pattern.push_back(pair<int, Point3f*>(58, point58));
        pattern.push_back(pair<int, Point3f*>(59, point59));
        pattern.push_back(pair<int, Point3f*>(60, point60));
        pattern.push_back(pair<int, Point3f*>(61, point61));
        pattern.push_back(pair<int, Point3f*>(62, point62));
        pattern.push_back(pair<int, Point3f*>(63, point63));
        pattern.push_back(pair<int, Point3f*>(64, point64));
        pattern.push_back(pair<int, Point3f*>(65, point65));
        pattern.push_back(pair<int, Point3f*>(66, point66));
        pattern.push_back(pair<int, Point3f*>(67, point67));
        pattern.push_back(pair<int, Point3f*>(68, point68));
        pattern.push_back(pair<int, Point3f*>(69, point69));
        pattern.push_back(pair<int, Point3f*>(70, point70));
        pattern.push_back(pair<int, Point3f*>(71, point71));
        pattern.push_back(pair<int, Point3f*>(72, point72));
        pattern.push_back(pair<int, Point3f*>(73, point73));
        pattern.push_back(pair<int, Point3f*>(74, point74));
        pattern.push_back(pair<int, Point3f*>(75, point75));
        pattern.push_back(pair<int, Point3f*>(76, point76));
        pattern.push_back(pair<int, Point3f*>(77, point77));
        pattern.push_back(pair<int, Point3f*>(78, point78));
        pattern.push_back(pair<int, Point3f*>(79, point79));
        pattern.push_back(pair<int, Point3f*>(80, point80));
        pattern.push_back(pair<int, Point3f*>(81, point81));
        pattern.push_back(pair<int, Point3f*>(82, point82));
        pattern.push_back(pair<int, Point3f*>(83, point83));
        pattern.push_back(pair<int, Point3f*>(84, point84));
    };

    static vector<pair<int, Point3f*>> getPattern3Dapril() {
        return pattern;
    };
//};

#endif //CAMERACALIBRATIONTOOL_3DPATTERN_H
