// =====================================================================================
// 
//       Filename:  sickday.h
// 
//    Description:  
// 
//        Version:  1.0
//        Created:  09/26/2010 02:08:25 AM
//       Revision:  none
//       Compiler:  g++
// 
//         Author:  Jan Elseberg (), jelseber@uos.de
//        Company:  Universitaet Osnabrueck
// 
// =====================================================================================

const int MIN_NR_PTS = 1200;

const double IMG_RES = 2.0; // in cm
const int BOARD_SIZE_X = 59.40 / IMG_RES;         // number of pixels wide
const int BOARD_SIZE_Z = 84.10 / IMG_RES;         // number of pixels high
      
const int WHITE_BORDER = 4;                       // number of pixels on the border to whiten. 10 cm ?

const double MINREFL = -10.0; 
const double MAXREFL = 10.0;
const double REFL_THRESHOLD = (0.0 - MINREFL ) / ( MAXREFL - MINREFL);   // threshold for binarizing image

const double MAX_DIST_TO_PLANE = 10.0;            // RANSAC max distance to plane
const double MIN_SCORE = 0.65;                    // minimal score for accepting the board hypothesis (between 0 and 1)
const double SCORE_SCALE = 0.50;                  // score is scaled so that SCORE_SCALE is probability 0 
     
///////////////  constants for OCR
const string OCRRESULT = "/tmp/ocrresult.txt";
const string OCRERROR = "/tmp/ocrerr.txt";
/////////////// number (except 1) are usually 17,27 pixel wide,high
//
const int MIN_NR_WIDTH = 28.0 / IMG_RES;          // 14 pixel ?  All Numbers except 1 must at least be this wide
const int MIN_1_WIDTH = 16.0 / IMG_RES;           // 8 pixel ?  Number 1 must at least be this wide
const int MIN_NR_HEIGHT = 46.0 / IMG_RES;         // 23 pixel ? Numbers must at least be this high

const int MAX_1_WIDTH = 28.0 / IMG_RES;           // 14 pixel ?  Number 1 must at most be this wide
const int MAX_NR_WIDTH = 42.0 / IMG_RES;          // 21 pixel ?  All Numbers except 1 must at most be this wide
const int MAX_NR_HEIGHT = 62.0 / IMG_RES;         // 31 pixel ? Numbers must at most be this high



