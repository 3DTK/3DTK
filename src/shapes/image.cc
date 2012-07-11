/*
 * image implementation
 *
 * Copyright (C) Dorit Borrmann
 *
 * Released under the GPL version 3.
 *
 */

#include "shapes/image.h"
#include "slam6d/globals.icc"
#include <limits>
#include <algorithm>
#include <string>
#include <fstream>
using std::string;
using std::ofstream;
#include <set>
using std::set;
#include <iterator>
using std::insert_iterator;

Image::Image(float _minw, float _maxw, float _minh, float _maxh, float
_resolution, const vector <Point> *points) {
  minh = _minh;
  minw = _minw;
  maxh = _maxh;
  maxw = _maxw;
  width = (maxw - minw)/_resolution + 1;
  height = (maxh - minh)/_resolution + 1;
  int size = points->size();

  data = new float*[width];
  for(int i = 0; i < width; i++) {
    data[i] = new float[height];
    for(int j = 0; j < height; j++) {
      data[i][j] = 0.0;
    }
  }
  if(points != 0) {
    double *tmpp = new double[3];
    double reflectance;
    double * polar = new double[3];
    for(int i = 0; i < size; i++) {
      Point p = (*points)[i];
      tmpp[0] = p.x;
      tmpp[1] = p.z;
      tmpp[2] = p.y;
      reflectance = p.reflectance;
      //tmpp = points[i];
      // phi theta rho
      // theta = width, phi = height, rho = distance value
      toPolar(tmpp, polar);
      int w_i = (int)((double)((deg(polar[1]) + 90.0) - minw)/(double)(maxw - minw) * (double)width) % width;
      //int w_i = ((double)((int)(deg(polar[1]) + 90)%360 - minw)/(double)(maxw - minw)) * (double)width;
      if(w_i < 0) {
        w_i = 0;
      } else if(w_i >= width) {
        w_i = width - 1;
      }
      int h_i = (int)((double)(deg(polar[0]) - minh)/(double)(maxh - minh) * (double)height);
      if(h_i < 0) {
        h_i = 0;
      } else if(h_i >= height) {
        h_i = height - 1;
      }
      
      if((fabs(data[w_i][h_i]) < 0.000001) || (data[w_i][h_i] < p.reflectance)) {
        data[w_i][h_i] = (float)reflectance;
        //cout << data[w_i][h_i] << " " << endl;
      }
      /*
      if((fabs(data[w_i][h_i]) < 0.000001) || (data[w_i][h_i] > polar[2])) {
        data[w_i][h_i] = (float)reflectance;
        //cout << data[w_i][h_i] << " " << endl;
      }
      */
    }
    delete[] tmpp;
    delete[] polar;
  }
  
}

Image::Image(float _minw, float _maxw, float _minh, float _maxh, float _resolution, double * const* points, int size) {
  minh = _minh;
  minw = _minw;
  maxh = _maxh;
  maxw = _maxw;
  width = (maxw - minw)/_resolution + 1;
  height = (maxh - minh)/_resolution + 1;
  
  data = new float*[width];
  for(int i = 0; i < width; i++) {
    data[i] = new float[height];
    for(int j = 0; j < height; j++) {
      data[i][j] = 0.0;
    }
  }
  if(points != 0) {
    double *tmpp = new double[3];
    double * polar = new double[3];
    for(int i = 0; i < size; i++) {
      tmpp[0] = points[i][0];
      tmpp[1] = points[i][2];
      tmpp[2] = points[i][1];
      //tmpp = points[i];
      // phi theta rho
      // theta = width, phi = height, rho = distance value
      toPolar(tmpp, polar);
      int w_i = (int)((double)((deg(polar[1]) + 90.0) - minw)/(double)(maxw - minw) * (double)width) % width;
      //int w_i = ((double)((int)(deg(polar[1]) + 90)%360 - minw)/(double)(maxw - minw)) * (double)width;
      if(w_i < 0) {
        w_i = 0;
      } else if(w_i >= width) {
        w_i = width - 1;
      }
      int h_i = (int)((double)(deg(polar[0]) - minh)/(double)(maxh - minh) * (double)height);
      if(h_i < 0) {
        h_i = 0;
      } else if(h_i >= height) {
        h_i = height - 1;
      }
      
      if((fabs(data[w_i][h_i]) < 0.000001) || (data[w_i][h_i] > polar[2])) {
        data[w_i][h_i] = (float)polar[2];
        //cout << data[w_i][h_i] << " " << endl;
      }
    }
    delete[] tmpp;
    delete[] polar;
  }
  
}

Image::Image(float _minw, float _maxw, float _minh, float _maxh, float _resolution, const vector<double*> *points) {
  //TODO correct this!!!
  minh = _minh;
  minw = _minw;
  maxh = _maxh;
  maxw = _maxw;
  width = (maxw - minw)/_resolution + 1;
  height = (maxh - minh)/_resolution + 1;
  
  data = new float*[width];
  for(int i = 0; i < width; i++) {
    data[i] = new float[height];
    for(int j = 0; j < height; j++) {
      data[i][j] = 0.0;
    }
  }
  if(points != 0) {
    double *tmpp;
    double * polar = new double[3];
    for(int i = 0; i < points->size(); i++) {
      tmpp = new double[3];
      //tmpp = (*points)[i];
      tmpp[0] = (*points)[i][0];
      tmpp[1] = (*points)[i][2];
      tmpp[2] = (*points)[i][1];
      // phi theta rho
      // theta = width, phi = height, rho = distance value
      toPolar(tmpp, polar);
      int w_i = (((int)(deg(polar[1]) + 180)%360 - minw)/(maxw - minw)) * width;
      w_i = (((int)(deg(polar[1]) ) - minw)/(maxw - minw)) * width;
      if(w_i < 0) {
        w_i = 0;
      } else if(w_i >= width) {
        w_i = width - 1;
      }
      int h_i = (((int)(deg(polar[0]) + 90)%180 - minh)/(maxh - minh)) * height;
      h_i = (((int)(deg(polar[0]) ) - minh)/(maxh - minh)) * height;
      if(h_i < 0) {
        h_i = 0;
      } else if(h_i >= height) {
        h_i = height - 1;
      }
      
      if((fabs(data[w_i][h_i] - 0.0) < 0.000100) || (data[w_i][h_i] > polar[2])) {
        data[w_i][h_i] = (float)polar[2];
      }
    }
  }
  for(int j = 0; j < height; j++) {
    for(int i = 0; i < width; i++) {
      cout << data[i][j] << " " ;
    }
    cout << endl;
  }
  
}

Image::~Image() {
  for(int i = 0; i < width; i++) {
    delete[] data[i];
  }
  delete[] data;
}

int Image::getWidth() {
  return width; 
}

int Image::getHeight() {
  return height;
}

float Image::getValueAt(int width, int height) {
  return data[width][height];
}

float Image::getMin() {
  float min = std::numeric_limits<float>::max();
  
  for(int i = 0; i < width; i++) {
    for(int j = 0; j < height; j++) {
      min = std::min(min, data[i][j]);
    }
  }
  return min;
}

float Image::getMax() {
  float max = std::numeric_limits<float>::min();
  for(int i = 0; i < width; i++) {
    for(int j = 0; j < height; j++) {
      max = std::max(max, data[i][j]);
    }
  }
  return max;
}

void Image::mergeregion(int x, int y, int delreg, int targetreg, int** img, int it) {
  cout << x << " " << y << " " << delreg << " " << targetreg << " " << it << endl;
  // traversed more than necessary
  if(it > 0) return;
//  cout << (j % width) << ", " << j / width << "   " << width << "   " <<  old<< " ->> " << targetreg << endl;
  img[x][y] = targetreg;
  cout << "A" << endl;
  // left
  if ( x > 0 && (img[x - 1][y] == delreg)){
    cout << "1" << endl;
    mergeregion(x-1, y ,delreg, targetreg, img);
    cout << "2" << endl;
  }
  cout << "B" << endl;
  // right
  if ( x < width - 1  && (img[x + 1][y] == delreg)){
    mergeregion(x+1, y, delreg, targetreg, img);
  }
  cout << "C" << endl;
  // up
  if ( y > 0 && (img[x][y-1] == delreg )) {
    mergeregion(x, y - 1, delreg, targetreg, img);
  }
  cout << "D" << endl;
  // down
  if ( y + 1 < height && (img[x][y+1] == delreg )) {
    mergeregion(x, y + 1, delreg, targetreg, img);
  }
  cout << "E" << endl;
}

int Image::calcMarker(float _threshold, int ** regdat) {
  // segment picture
  threshold = _threshold;
  cout << threshold << " ";
  int counter = 0;
  for(int y = 0; y < height; y++) {
    for(int x = 0; x < width; x++) {
      if(data[x][y] > threshold) {
        counter++;
        regdat[x][y] = 100;
        //data[x][y] = 100;
      } else {
        regdat[x][y] = 0;
        //data[x][y] = 0;
      }
    }
  }

  //printScans(regdat, height, width)
  //printImage("marker.ppm", false, regdat, height, width, 0, 1);
  printImage("marker.ppm", false, regdat, height, width, 0, 100);
  cout << counter << endl;
  return counter;
}

int Image::cluster(float dist, int ** dat, int ** regdat) {
  vector<set<int> > linked;
  int up, left;
  int region = 0;
  for(int y = 0; y < height; y++) {
    for(int x = 0; x < width; x++) {
      float value = dat[x][y];
      
      if(value != 0) {
        // left border
        if(x==0) {
          left = value + dist*2.0;
        } else {
          left = dat[x-1][y];
        }
        // upper border
        if(y==0) {
          up = value + dist*2.0;
        } else {
          up = dat[x][y-1];
        }
      
        if (fabs(left - value) > dist) {
          if (fabs(up - value) > dist) {
            regdat[x][y] = ++region;                           // new region
            set<int> *joined = new set<int>;
            joined->insert(region);
            linked.push_back(*joined);
          } else {
            regdat[x][y] = regdat[x][y - 1];  // use upper region
          }
        } else if (fabs(up - value) > dist) {
          regdat[x][y] = regdat[x-1][y];                      // use left region
        } else {
          // merge left and upper region, default to lower regioning number
          if (regdat[x][y - 1] == regdat[ x - 1][y]) {
            regdat[x][y] = regdat[x-1][y];
            continue; // already same region, nothing to do
          }

          /*
          int minreg, maxreg;
          falschrum??
          minreg = regdat[x][y - 1];
          maxreg = regdat[x - 1][y];
          */
          int up = regdat[x][y - 1];
          int left = regdat[x - 1][y];
          regdat[x][y] = up;
          set<int> current;
          set<int> *joined = new set<int>;
          joined->insert(up);
          joined->insert(left);
          
          for(vector<set<int> >::iterator itr = linked.begin(); itr != linked.end(); ) {
            current = (*itr);
            if(current.find(up) != current.end() || current.find(left) != current.end()) {
              set<int> *newjoined = new set<int>();
              insert_iterator<set<int> > res_ins(*newjoined, newjoined->begin());

              set_union(joined->begin(), joined->end(), current.begin(), current.end(), res_ins); 
              joined->clear();
              delete joined;
              joined = newjoined;
              itr=linked.erase(itr);
            } else {
              itr++;
            }
          }

          linked.push_back(*joined);
      }
    }
    }
  }

  cout << region << " " << linked.size();
  int linkedindex[region+1];
  linkedindex[0] = 0;
  set<int> current;
  int link = 0;
  for(vector<set<int> >::iterator itr = linked.begin(); itr != linked.end(); itr++) {
    current = (*itr);
    for(set<int>::iterator sitr = current.begin(); 
        sitr != current.end(); 
        sitr++) {
      linkedindex[*sitr] = link;
    }
    link++;
  }

  
  cout << "Regions "  << linked.size() << endl;
  /*
  for(int i = 0; i <= region; i++) {
    cout << i << " " << linkedindex[i] << endl;
  }
  */
  for(int y = 0; y < height; y++) {
    for(int x = 0; x < width; x++) {
      regdat[x][y] = linkedindex[regdat[x][y]];
    }
  }
  
  printImage("cluster.ppm", false, regdat, height, width, 0, linked.size());
 
  return linked.size();

}

void Image::writeCenters(int regions, int** cluster, const vector<Point> *points) {
  int size = points->size();

  vector<Point> clusters[regions];
  /*
  vector<vector<Point> > clusters; 
  
  
  for(int i = 0; i < regions; i++) {
    vector<Point> tmp = new vector<Point>();
    clusters.push_back(tmp);
  }
  
  
  cout << "Regions: " << regions << " " << clusters.size() << endl;;
  for(int i = 0; i < regions; i++) {
    cout << i << " " << clusters[i]->size() << endl;
    Point p = *clusters[i][0];
    cout << p.x << " " << p.y << " " << p.z << endl;
  }
  cout << "done" << endl;
  */
  /*
  for(int i = 0; i < regions; i++) {
    vector<Point> *tmp = new vector<Point>();
    clusters[i].push_back(tmp);
  }
  */

  if(points != 0) {
    double *tmpp = new double[3];
    double reflectance;
    double * polar = new double[3];
    for(int i = 0; i < size; i++) {
      Point p = (*points)[i];
      tmpp[0] = p.x;
      tmpp[1] = p.z;
      tmpp[2] = p.y;
      reflectance = p.reflectance;
      //tmpp = points[i];
      // phi theta rho
      // theta = width, phi = height, rho = distance value
      if(reflectance > threshold) {
        toPolar(tmpp, polar);
        int w_i = (int)((double)((deg(polar[1]) + 90.0) - minw)/(double)(maxw - minw) * (double)width) % width;
        //int w_i = ((double)((int)(deg(polar[1]) + 90)%360 - minw)/(double)(maxw - minw)) * (double)width;
        if(w_i < 0) {
          w_i = 0;
        } else if(w_i >= width) {
          w_i = width - 1;
        }
        int h_i = (int)((double)(deg(polar[0]) - minh)/(double)(maxh - minh) * (double)height);
        if(h_i < 0) {
          h_i = 0;
        } else if(h_i >= height) {
          h_i = height - 1;
        }
        int tmp = cluster[w_i][h_i] - 1;
        if(cluster[w_i][h_i] != 0) {
          clusters[tmp].push_back(p);
        }
      }
    }
    delete[] tmpp;
    delete[] polar;
  }

  cout << "Regions: " << regions << " " << clusters->size() << endl;
  int markercounter = 0;
  for(int i = 0; i < regions; i++) {
    //cout << i << " " << clusters[i].size() << endl;
    double point[3];
    point[0] = point[1] = point[2] = 0;
    for(int j = 0; j < clusters[i].size(); j++) {
      point[0] += clusters[i][j].x;
      point[1] += clusters[i][j].y;
      point[2] += clusters[i][j].z;
    }
    for(int j = 0; j < 3; j++) {
      point[j] /= clusters[i].size();
    }
    //cout << p.x << " " << p.y << " " << p.z << endl;
    cout << point[0] << " " << point[1] << " " << point[2] << " TP" << to_string(markercounter, 4) << endl;
    markercounter++;
  }
  
}

int Image::blobColor(float dist, int ** regdat) {
  // segment picture
  int region = 0; // start at 12 to incresae visibility 
  float up, left;

  for(int y = 0; y < height; y++) {
    for(int x = 0; x < width; x++) {

      float value = data[x][y];
      // left border
      if (x == 0 ) {
        left = value + dist*2.0;
      } else {
        left = data[x - 1][y];
      }
      // or upper border
      if ( y == 0) {
        up = value + dist*2;
      } else {
        up = data[x][y - 1];
      }

      if (fabs(left - value) > dist) {
        if (fabs(up - value) > dist) {
          regdat[x][y] = ++region;                           // new region
        } else {
          regdat[x][y] = regdat[x][y - 1];  // use upper region
        }
      } else if (fabs(up - value) > dist) {
        regdat[x][y] = regdat[x-1][y];                      // use left region
      } else {
        // merge left and upper region, default to lower regioning number
        if (regdat[x][y - 1] == regdat[ x - 1][y]) {
          regdat[x][y] = regdat[x-1][y];
          continue; // already same region, nothing to do
        }

        int minreg, maxreg;
        /*
        falschrum??
        minreg = regdat[x][y - 1];
        maxreg = regdat[x - 1][y];
        */
        maxreg = regdat[x][y - 1];
        minreg = regdat[x - 1][y];
        regdat[x][y] = maxreg;
        cout << "mergeRegion" << endl;
        mergeregion(x, y, maxreg, minreg, regdat);
        cout << "mergeRegion done" << endl;
      }
    }
  }

  cout << "color done" << endl;
  //printScans(regdat, height, width)
  printImage("bla.ppm", false, regdat, height, width, 1.0, region);
  return region;
}

void kMeans(float dist, int ** regdat) {

}

void EM(float dist, int ** regdat) {

}

void normalizedCuts(float dist, int ** regdat) {

}

void lineMerge() {
  
}

void Image::printScans(int ** regdat, double * const* points, int size) {
  int max_i = 0;
  for(int i = 0; i < width; i++) {
    for(int j = 0; j < height; j++) {
      max_i = max_i > regdat[i][j] ? max_i : regdat[i][j];
    }
  }
  cout << "size " << size << endl;
  cout << "max_i is " << max_i << endl;
  vector<double*> clusters[max_i + 1];
  
  if(points != 0) {
    cout << "points" << endl;
    double *tmpp = new double[3];
    double * polar = new double[3];
    for(int i = 0; i < size; i++) {
      tmpp[0] = points[i][0];
      tmpp[1] = points[i][2];
      tmpp[2] = points[i][1];
      //tmpp = points[i];
      // phi theta rho
      // theta = width, phi = height, rho = distance value
      toPolar(tmpp, polar);
      int w_i = (int)((double)((deg(polar[1]) + 90.0) - minw)/(double)(maxw - minw) * (double)width) % width;
      //int w_i = ((double)((int)(deg(polar[1]) + 90)%360 - minw)/(double)(maxw - minw)) * (double)width;
      if(w_i < 0) {
        w_i = 0;
      } else if(w_i >= width) {
        w_i = width - 1;
      }
      int h_i = (int)((double)(deg(polar[0]) - minh)/(double)(maxh - minh) * (double)height);
      if(h_i < 0) {
        h_i = 0;
      } else if(h_i >= height) {
        h_i = height - 1;
      }
      //cout << i << " " << regdat[w_i][h_i] <<  endl; 
      clusters[regdat[w_i][h_i]].push_back(points[i]);
    }
    delete[] tmpp;
    delete[] polar;
    
    cout << "clustering done " << endl;

    string clusteroutname = "";
    int index = 0;
    for(int i = 0; i < max_i; i++) {
      if(clusters[i].size() > 0) {
        clusteroutname = "clusters/scan" + to_string(index, 3) + ".3d";  
        index++;
        ofstream clusterout(clusteroutname.c_str());
        clusterout << i << endl;
        for(int j = 0; j < clusters[i].size(); j++) {
          clusterout << clusters[i][j][0] << " "
          << clusters[i][j][1] << " " << clusters[i][j][2] << endl;
        }
        clusterout.close();
        clusterout.clear();
      }
    }
  }
}
      

void Image::printImage(const char* filename, bool color, int** img, int height, int width, int min, int max) {
  double Fact = 255.0 /(max - min);
  std::ofstream outfile(filename);
  if(outfile.good()) {
    if(!color) {
      outfile << "P2\n" << width << " " << height << std::endl << "255" << endl;
    } else {
      outfile << "P3\n" << width << " " << height << std::endl << "255" << endl;
    }

    for(int j = 0; j < height; j++) {
      for(int i = 0; i < width; i++) {
        short gray = std::min(std::max((int)(Fact * (img[i][j] - min)), 0), 255); 
        if(!color) {
          outfile << gray << " ";
        } else {
          double h = 255.0-gray;
          double v = 1.0;
          double s = 1.0;
          double r = 0, g = 0, b = 0;
          double h1 = h/60.0;
          int i = (int)h1;
          double f = h1 - i;
          double pv = v * (1-s);
          double qv = v * (1-s*f);
          double tv = v * (1-s*(1-f));
          switch(i) {
            case 0:
              r = v;
              g = tv;
              b = pv;
              break;
            case 1:
              r = qv;
              g = v;
              b = pv;
              break;
            case 2:
              r = pv;
              g = v;
              b = tv;
              break;
            case 3:
              r = pv;
              g = qv;
              b = v;
              break;
            case 4:
              r = tv;
              g = pv;
              b = v;
              break;
            case 5:
              r = v;
              g = pv;
              b = qv;
              break;
            case 6:
              r = v;
              g = tv;
              b = pv;
              break;
            default:
              break;
          }
          outfile << (int)(r*255.0) << " " << (int)(g*255.0) << " " << (int)(b*255.0) << " "; 
        }
      }
      outfile << std::endl;
    }
  }
  outfile.close();

}

void Image::printImage(const char *filename, bool color) {
  float min = getMin();
  float max = getMax();

  double Fact = 255.0 /(max - min);
  std::ofstream outfile(filename);
  if(outfile.good()) {
    if(!color) {
      outfile << "P2\n" << width << " " << height << std::endl << "255" << endl;
    } else {
      outfile << "P3\n" << width << " " << height << std::endl << "255" << endl;
    }

    for(int j = 0; j < height; j++) {
      for(int i = 0; i < width; i++) {
        short gray = std::min(std::max((int)(Fact * (data[i][j] - min)), 0), 255); 
        if(!color) {
          outfile << gray << " ";
        } else {
          double h = 255.0-gray;
          double v = 1.0;
          double s = 1.0;
          double r = 0, g = 0, b = 0;
          double h1 = h/60.0;
          int i = (int)h1;
          double f = h1 - i;
          double pv = v * (1-s);
          double qv = v * (1-s*f);
          double tv = v * (1-s*(1-f));
          switch(i) {
            case 0:
              r = v;
              g = tv;
              b = pv;
              break;
            case 1:
              r = qv;
              g = v;
              b = pv;
              break;
            case 2:
              r = pv;
              g = v;
              b = tv;
              break;
            case 3:
              r = pv;
              g = qv;
              b = v;
              break;
            case 4:
              r = tv;
              g = pv;
              b = v;
              break;
            case 5:
              r = v;
              g = pv;
              b = qv;
              break;
            case 6:
              r = v;
              g = tv;
              b = pv;
              break;
            default:
              break;
          }
          outfile << (int)(r*255.0) << " " << (int)(g*255.0) << " " << (int)(b*255.0) << " "; 
        }
      }
      outfile << std::endl;
    }
  }
  outfile.close();
}
