// =====================================================================================
// 
//       Filename:  integralimg.cc
// 
//    Description:  
// 
//        Version:  1.0
//        Created:  09/20/2010 06:58:34 PM
//       Revision:  none
//       Compiler:  g++
// 
//         Author:  Jan Elseberg (), jelseber@uos.de
//        Company:  Universitaet Osnabrueck
// 
// =====================================================================================
//
#include <iostream>
using std::cout;
using std::cerr;
using std::endl;


#ifndef __INTEGRALIMG_H__
#define __INTEGRALIMG_H__

#define TBB 5
#define SBB 7
#define HEIGHT 26 
//#define HEIGHT 40 
#define WIDTH 30 
const int black[4][2] = { {0,0}, {WIDTH+2*SBB,0}, {WIDTH+2*SBB,HEIGHT+TBB}, {0,HEIGHT+TBB}};
const int white[4][2] = { {SBB,TBB}, {WIDTH+SBB,TBB}, {WIDTH+SBB,HEIGHT+TBB}, {SBB,HEIGHT+TBB}};
/*const int black[4][2] = { {0,0}, {30+2*SBB,0}, {30+2*SBB,40+TBB}, {0,40+TBB}};
const int white[4][2] = { {SBB,TBB}, {30+SBB,TBB}, {30+SBB,40+TBB}, {SBB,40+TBB}};
*/
class integral_img {
public:

  integral_img(int **_img, int x, int y) {
    img = _img;
    X = x;
    Y = y;

    for (int i = 1; i < X; i++) {
        img[i][0] += img[i-1][0];
    }
    for (int j = 1; j < Y; j++) {
        img[0][j] += img[0][j-1];
    }

    for (int i = 1; i < X; i++) {
      for (int j = 1; j < Y; j++) {
        img[i][j] += img[i-1][j] + img[i][j-1] - img[i-1][j-1];
      }
    }
  }

  int At(int x, int y) {
    if (x < 0 || y < 0) {
      return 0;      
    }
    if (x >= X) x = X - 1;
    if (y >= Y) y = Y - 1;
    return img[x][y];
  }

  double getBest(int &x, int&y) {
    int score;
    int maxscore = 0;
    int maxx = 0;
    int maxy = 0;


    for (int i = -SBB; i < X+SBB; i++) {
      for (int j = -TBB; j < Y+TBB; j++) {
        score = 2*getWhite(i,j) - getBlack(i,j);
//        cout << i << " " << j << ": W " << getWhite(i,j)<< " B " << getBlack(i,j) << " SCORE " << score << endl;

        if (score > maxscore) {
          maxscore = score;
          maxx = i;
          maxy = j;
        }
      }
    }
    x = maxx + SBB;
    y = maxy + TBB;  
    return (double)maxscore/(double)(HEIGHT*WIDTH);
  }

  inline int getWhite(int x, int y) {
    return At(x + white[0][0], y + white[0][1]) + At(x + white[2][0], y + white[2][1]) -
           At(x + white[1][0], y + white[1][1]) - At(x + white[3][0], y + white[3][1]);
  }
  
  inline int getBlack(int x, int y) {
    return At(x + black[0][0], y + black[0][1]) + At(x + black[2][0], y + black[2][1]) -
           At(x + black[1][0], y + black[1][1]) - At(x + black[3][0], y + black[3][1]);
  }


  int **img;
  int X;
  int Y;

};

#endif
