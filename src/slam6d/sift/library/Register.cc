/**
 * @file Register.cc
 * @brief Implementation of Register class
 * @author Andreas Nuechter. Jacobs University Bremen gGmbH, Grmany.
 * @author HamidReza Houshiar. Jacobs University Bremen gGmbH, Germany.
 * @author Darko Makreshanski. Jacobs University Bremen gGmbH, Germany.
 */
#include "slam6d/sift/library/Register.h"
#include "slam6d/sift/library/Coord.h"
#include <stdlib.h>
#include <iostream>
#include <cassert>
#include <limits>
#include <list>
#include <sstream>
#define _USE_MATH_DEFINES
#include <cmath>
#define M_PI 3.1415

#include "slam6d/icp6Dquat.h"
#include "slam6d/sift/library/PolarPointCloud.h"
#include "slam6d/sift/library/PointCloud.h"

using namespace std;

int mc = 0;
int maxd[1000];

bool getCoord (FeatureBase feat, PanoramaMap *map, Coord &c)
{
  int x = (int) feat.x;
  int y = map->height - (int) feat.y - 1;
  
  if (x > map->width - 1) x = map->width - 1;
  if (x < 0) x = 0;
  if (y > map->height - 1) y = map->height - 1;
  if (y < 0) y = 0;
  
  Coord tmp;
  int index = 0;
  double depth = 0;
  
  if (map->data[x][y].meta != 0) 
    {
      index++;
      depth += map->data[x][y].meta;
    }
  
  //double a = (map->maxa - map->mina) * (((float) x) / (float) map->width) + map->mina;
  //double b = (map->maxb - map->minb) * (((float) y) / (float) map->height) + map->minb;
  
  if (depth != 0)
    {
      //c = Coord(POLAR, a / 360.0 * 2 * M_PI, b / 360.0 * 2 * M_PI, depth);
      //c = Coord(map->data[x][y].x, map->data[x][y].z, -map->data[x][y].y);
      c = Coord(map->data[x][y].x, map->data[x][y].y, map->data[x][y].z);
      
      c.depth = depth;
      return true;
    }
  
  //if point falls into some empty spot get depth from surrounding
  float limit = 8 * feat.scale;
  for (int rad = 4 ; rad < limit ; rad += limit / 4) 
    {    
      int rx, ry;
      
      rx = x - rad;
      ry = y - rad;
      if (!(rx < 0 || ry < 0) )
	{
	  if (map->data[rx][ry].meta != 0)
	    {
	      index++;
	      //tmp = Coord(map->data[rx][ry].x, map->data[rx][ry].z, -map->data[rx][ry].y);
	      tmp = Coord(map->data[rx][ry].x, map->data[rx][ry].y, map->data[rx][ry].z);
	      c = c + tmp;
	      depth += map->data[rx][ry].meta;
	    }
	}
      
      rx = x - rad;
      ry = y + rad;
      if (!(rx < 0 || ry > map->width -1) )
	{
	  if (map->data[rx][ry].meta != 0)
	    {
	      index++;
	      //tmp = Coord(map->data[rx][ry].x, map->data[rx][ry].z, -map->data[rx][ry].y);
	       tmp = Coord(map->data[rx][ry].x, map->data[rx][ry].y, map->data[rx][ry].z);
	      c = c + tmp;
	      depth += map->data[rx][ry].meta;
	    }
	}
      
      rx = x + rad;
      ry = y + rad;
      if (!(rx > map->width - 1 || ry > map->width -1) )
	{
	  if (map->data[rx][ry].meta != 0)
	    {
	      index++;
	      //tmp = Coord(map->data[rx][ry].x, map->data[rx][ry].z, -map->data[rx][ry].y);
	      tmp = Coord(map->data[rx][ry].x, map->data[rx][ry].y, map->data[rx][ry].z);
	      c = c + tmp;
	      depth += map->data[rx][ry].meta;
	    }
	}
      
      rx = x + rad;
      ry = y - rad;
      if (!(rx > map->width - 1 || ry < 0) )
	{
	  if (map->data[rx][ry].meta != 0)
	    {
	      index++;
	      //tmp = Coord(map->data[rx][ry].x, map->data[rx][ry].z, -map->data[rx][ry].y);
	       tmp = Coord(map->data[rx][ry].x, map->data[rx][ry].y, map->data[rx][ry].z);
	      c = c + tmp;
	      depth += map->data[rx][ry].meta;
	    }
	}
      
      if (index != 0)
	{
	  depth /= index;
	  //c = Coord(POLAR, a / 360.0 * 2 * M_PI, b / 360.0 * 2 * M_PI, depth);
	  c = c / index;
	  c.depth = depth;
	  return true;
	}
    }
  cout<<"DONE"<<endl;
  mc++;
  return false;
}

Register::Register(FeatureMatchSetGroup *group, std::vector<PanoramaMap*> &tmaps)
{
  for (int i = 0 ; i < 1000 ; i++)
    {
      maxd[i] = 0;
    }
  mode = MAXIMUM;
  mode_maximum = 100000;
  d = 2;
  t = 1.0;
  dinfluence = 0.05;
  //difluence = 5;
  mind = 2;
  //mind = 200;
  mina = 2.0 / 360.0 * 2 * M_PI;
  
  assert(group);
  vector<PanoramaMap*>::iterator ita;
  for (ita = tmaps.begin(); ita != tmaps.end() ; ita++)
    {
      assert((*ita));
      maps[(*ita)->scanid] = (*ita);
    }
  this->group = group;
}

void Register::registerScans()
{
  list< list<ScanTransform> >	 components;
  list<FeatureMatchSet>::iterator it;
  for (it = group->matchsets.begin() ; it != group->matchsets.end() ; it++)
    {
      if (maps.find(it->firstscan) == maps.end() || maps.find(it->secondscan) == maps.end())
	{
	  continue;
	}
      
      double tr[4][4];
      bool fits;
      mc = 0;
      fits = Register::registerSet(&(*it), maps[it->firstscan], maps[it->secondscan], tr);
      
      if (fits)
	{	  
	  bool foundfirst = false;
	  list< list<ScanTransform> >::iterator first_component;
	  list<ScanTransform>::iterator first_ind;
	  bool foundsecond = false;
	  list< list<ScanTransform> >::iterator second_component;
	  list<ScanTransform>::iterator second_ind;
	  
	  for (list< list<ScanTransform> >::iterator f = components.begin() ; f != components.end() ; f++)
	    {
	      for (list<ScanTransform>::iterator s = f->begin() ; s != f->end() ; s++)
		{
		  if (s->scanid == it->firstscan)
		    {
		      foundfirst = true;
		      first_component = f;
		      first_ind = s;
		    }
		  if (s->scanid == it->secondscan)
		    {
		      foundsecond = true;
		      second_component = f;
		      second_ind = s;
		    }
		}
	    }
	  
	  if (!foundfirst && !foundsecond)
	    {
	      list<ScanTransform> newcomponent;
	      ScanTransform first(it->firstscan);
	      ScanTransform second(it->secondscan, tr);
	      newcomponent.push_back(first);
	      newcomponent.push_back(second);
	      components.push_back(newcomponent);     
	    } else
	    if (foundfirst && foundsecond)
	      {
		if (first_component != second_component)
		  {
		    //merge components
		    for (list<ScanTransform>::iterator ni = second_component->begin() ; ni != second_component->end() ; ni++)
		      {
			ScanTransform secondtr = first_ind->multiply(ScanTransform("", tr).multiply(second_ind->inverse().multiply(*ni)));
			secondtr.scanid = ni->scanid;
			first_component->push_back(secondtr);			
		      }
		    components.erase(second_component);
		  }
	      } else
	      if (foundfirst)
                {
		  //add second
		  ScanTransform secondtr = first_ind->multiply(ScanTransform("", tr));
		  secondtr.scanid = it->secondscan;
		  first_component->push_back(secondtr);
		} else
		if (foundsecond)
		  {
		    //add first
		    ScanTransform firsttr = second_ind->multiply(ScanTransform("", tr).inverse());
		    firsttr.scanid = it->firstscan;
		    second_component->push_back(firsttr); 
		  }
	}
    }
  
  cout << "Number of components: " << components.size() << endl;
  int i = 1;
  for (list< list<ScanTransform> >::iterator f = components.begin() ; f != components.end() ; f++)
    {
      cout << "Component " << i++ << ": ";
      for (list<ScanTransform>::iterator s = f->begin() ; s != f->end() ; s++) 
	{
	  cout << s->scanid << " ";
	  string foutdat = s->scanid + ".dat";
	  string fout = s->scanid + ".frames";
	  ofstream f(fout.c_str());
	  ofstream dat(foutdat.c_str());
	  dat << s->scanid << endl;
	  for (int n = 0 ; n < 2 ; n++)
	    {
	      for (int y = 0 ; y < 4 ; y++)
		{
		  for (int x = 0 ; x < 4 ; x++)
		    {
		      if(n == 0)
			dat << s->transform[y][x] << ((x == 3) ? "" : " ");
		      f << s->transform[x][y] <<" ";
		    }
		  if(n == 0)
		    dat << endl;
		}
	      f << "2" << endl;
	    }
	  f.close();
	  dat.close();
	}
      cout << endl;
    }
}

//transform a coordinate with the given align matrix from the icp6D_quat method
//only the 3x3 upper left matrix is considered because we are supplying the align method already centered triangles without the need for translation
Coord transformCoord(Coord c, double *m)
{
  Coord res;
  res.x = m[0]*c.x + m[4]*c.y + m[8]*c.z;
  res.y = m[1]*c.x + m[5]*c.y + m[9]*c.z;
  res.z = m[2]*c.x + m[6]*c.y + m[10]*c.z;
  return res;
}

void Register::processTrianglePair(FeatureMatchSet *set, PanoramaMap *map1, PanoramaMap *map2,
				   int p1, int p2, int p3, double &best_error, int &best_ind, int best_pt[3], double best_align[16], Coord &tr1, Coord &tr2,
				   vector<Coord> &coordsA,
				   vector<bool> &hasCoordsA,
				   vector<Coord> &coordsB,
				   vector<bool> &hasCoordsB)
{
  int countm = set->matches.size();

  Coord c1a, c2a, c3a, c1b, c2b, c3b;
  double c1ad[3], c2ad[3], c3ad[3];
  double c1bd[3], c2bd[3], c3bd[3];
  double c[3] = {0,0,0};

  if (!hasCoordsA[p1]) return;
  if (!hasCoordsA[p2]) return;
  if (!hasCoordsA[p3]) return;

  if (!hasCoordsB[p1]) return;
  if (!hasCoordsB[p2]) return;
  if (!hasCoordsB[p3]) return;

  c1a = coordsA[p1];
  c2a = coordsA[p2];
  c3a = coordsA[p3];
  
  c1b = coordsB[p1];
  c2b = coordsB[p2];
  c3b = coordsB[p3];
  
  double a12, a13, a23, b12, b13, b23;
  a12 = (c1a - c2a).abs(); a13 = (c1a - c3a).abs(); a23 = (c2a - c3a).abs();
  b12 = (c1b - c2b).abs(); b13 = (c1b - c3b).abs(); b23 = (c2b - c3b).abs();
  
  if (a12 < mind || a13 < mind || a23 < mind || b12 < mind || b13 < mind || b23 < mind)
    {
      return;
    }
  
  Coord centera = (c1a + c2a + c3a) / 3;
  Coord centerb = (c1b + c2b + c3b) / 3;

  (c1a - centera).toArray(c1ad); (c2a - centera).toArray(c2ad); (c3a - centera).toArray(c3ad);
  (c1b - centerb).toArray(c1bd); (c2b - centerb).toArray(c2bd); (c3b - centerb).toArray(c3bd);
  
  vector<PtPair> pairs;
  
  pairs.push_back(PtPair(c1ad, c1bd));
  pairs.push_back(PtPair(c2ad, c2bd));
  pairs.push_back(PtPair(c3ad, c3bd));
  
  double align[16];
  
  icp6D_QUAT q(true);
  q.Point_Point_Align(pairs, align, c, c);
  
  double inlier_error = 0;
  int inlier_error_ind = 0;
  for (int itm = 0 ; itm < countm ; itm++)
    {
      if (itm == p1 || itm == p2 || itm == p3)
	{
	  continue;
	}
      Coord tca, tcb;
      if (!hasCoordsA[itm]) continue;
      if (!hasCoordsB[itm]) continue;
      tca = coordsA[itm];
      tcb = coordsB[itm];

      Coord tcb_trans = centera + transformCoord(tcb - centerb, align);
      double errori = (tcb_trans - tca).abs();
      if (errori < t)
        {
	  cout<< "errori=            " << errori << endl;
	  inlier_error += errori;
	  inlier_error_ind++;
	  //cout<<inlier_error_ind<<endl;
	}
    }
  maxd[inlier_error_ind]++;
  if (inlier_error_ind >= d)
    {
      cout<<"inlier=                       "<<inlier_error_ind<<endl;
      double avg_error = inlier_error / inlier_error_ind;
      if (avg_error - dinfluence*inlier_error_ind < best_error - dinfluence*best_ind)
	{
	  best_error = avg_error;
	  best_ind = inlier_error_ind;
	  best_pt[0] = p1;
	  best_pt[1] = p2;
	  best_pt[2] = p3;
	  tr1 = centera;
	  tr2 = centerb;
	  for (int j = 0 ; j < 16 ; j++)
	    {
	      best_align[j] = align[j];
	    }
	}
    }
}

bool Register::registerSet(FeatureMatchSet *set, PanoramaMap *map1, PanoramaMap *map2, double trans[][4])
{
  cout << endl << endl << endl << "Registering: " << map1->scanid << " and " << map2->scanid << endl;
  int countm = set->matches.size();
  cout << set->firstscan << " " << set->secondscan << endl;
  cout << countm << endl;
  
  if (countm < 3 + d)
    {
      return false;
    }
  int countp =0;
  double best_error = numeric_limits<double>::max( );
  int best_ind = 0;
  int best_pt[3];
  Coord tr1, tr2;
  double best_align[16];
  
  vector<Coord> coordsA;
  vector<bool> hasCoordsA;
  vector<Coord> coordsB;
  vector<bool> hasCoordsB;
  
  for (int i = 0 ; i < countm ; i++)
    {
      Coord res;
      if (!getCoord(set->matches[i].first, map1, res))
	{
	  hasCoordsA.push_back(false);
	}
      else
        {
	  hasCoordsA.push_back(true);
	}
      coordsA.push_back(res);
      if (!getCoord(set->matches[i].second, map2, res))
	{
	  hasCoordsB.push_back(false);
	}
      else
        {
	  hasCoordsB.push_back(true);
	}
      coordsB.push_back(res);
    }

  if (mode == ALL)
    {
      cout << "Going through all points...\n";
      for (int p1 = 0 ; p1 < countm - 2 ; p1++) 
	{
	  for(int p2 = p1 + 1 ; p2 < countm - 1 ; p2++)
	    {
	      for (int p3 = p2 + 1 ; p3 < countm ; p3++)
		{
		  
		  processTrianglePair(set, map1, map2, p1, p2, p3,
				      best_error, best_ind, best_pt, best_align, tr1, tr2,
				      coordsA,
				      hasCoordsA,
				      coordsB,
				      hasCoordsB);
		}
	    }
	  cout << p1 + 1 << "/" << countm - 2 << endl;
	}
    }
  else
    {
      int iter;
      if (mode == MAXIMUM)
        {
	  iter = mode_maximum;
        }
      else
        {
	  iter = (countm * (countm - 1) * (countm - 2)) / 6 * mode_percentage;
        }
      int percent = iter / 100;
      cout << percent << endl;
      for (int it = 0 ; it < iter ; it++)
        {
	  int p1 = rand() % countm;
	  int p2 = rand() % countm;
	  int p3 = rand() % countm;
	  if (p1 == p2 || p1 == p3 || p2 == p3)
            {
	      continue;
            }
	  processTrianglePair(set, map1, map2, p1, p2, p3,
			      best_error, best_ind, best_pt, best_align, tr1, tr2,
			      coordsA,
			      hasCoordsA,
			      coordsB,
			      hasCoordsB);
	  if ((it % percent) == 0)
            {
	      //cout << "p1" << p1 << "p2" << p2 << "p3" << p3 << endl;
	      //cout << "best_error" << best_error << "best_ind" << best_ind << endl;
	      cout << (it / percent) + 1 << "%" << endl;
            }
        }
      cout << endl;
    }
  ostringstream strout;
  strout << "histogram_d-" << map1->scanid << "-" << map2->scanid << "_" << map1->width << "x" << map1->height << "-mm-" << mode_maximum;
  ofstream out(strout.str().c_str());
  for (int i = 0 ; i < 1000 ; i++)
    {
      out << i << " " << maxd[i] << endl;
    }
  out.close();
  if (best_ind == 0)
    {
      return false;
    }
  
  //	cout << endl << endl << endl << "Results from: " << map1->scanid << " and " << map2->scanid << endl << endl;
  
  cout << "Best: " << best_error << " from: " << best_ind << endl;
  cout << "Points: " << best_pt[0] << " " << best_pt[1] << " " << best_pt[2] << endl;
  
  double best_rot[9];
  best_rot[0] = best_align[0]; best_rot[3] = best_align[4]; best_rot[6] = best_align[8];
  best_rot[1] = best_align[1]; best_rot[4] = best_align[5]; best_rot[7] = best_align[9];
  best_rot[2] = best_align[2]; best_rot[5] = best_align[6]; best_rot[8] = best_align[10];
  
  Coord tr = transformCoord(Coord()-tr2, best_align) + tr1;
  //	Coord tr = tr1 - tr2;
  
  cout << "Translation: " << tr;
  
  cout << "Rotation: " << endl;
  cout << best_align[0] << " " << best_align[4] << " " << best_align[8] << endl; //" " << best_align[12] << endl;
  cout << best_align[1] << " " << best_align[5] << " " << best_align[9] << endl; //" " << best_align[13] << endl;
  cout << best_align[2] << " " << best_align[6] << " " << best_align[10] << endl; //" " << best_align[14] << endl;
  //	cout << best_align[3] << " " << best_align[7] << " " << best_align[11] << " " << best_align[15] << endl;
  
  cout << "countm: " << countm << endl;
  cout << "countp: " << countp << endl;
  
  Coord ca1, ca2, ca3;
  getCoord(set->matches[best_pt[0]].first, map1, ca1);
  getCoord(set->matches[best_pt[1]].first, map1, ca2);
  getCoord(set->matches[best_pt[2]].first, map1, ca3);
  
  Coord cb1, cb2, cb3;
  getCoord(set->matches[best_pt[0]].second, map2, cb1);
  getCoord(set->matches[best_pt[1]].second, map2, cb2);
  getCoord(set->matches[best_pt[2]].second, map2, cb3);
	
  //	cb1 = transformCoord(cb1, best_align);
  //	cb1 = cb1 + tr;
  //	cb2 = transformCoord(cb2, best_align);
  //	cb2 = cb2 + tr;
  //	cb3 = transformCoord(cb3, best_align);
  //	cb3 = cb3 + tr;
  
  cout << "Abs: " << (ca1 - ca2).abs() << " " << (ca1 - ca3).abs() << " " << (ca2 - ca3).abs() << endl;
  cout << ca1 << cb1 << ca2 << cb2 << ca3 << cb3;  
  cout << endl << tr1 << tr2 << endl;

  trans[0][0] = best_align[0]; trans[0][1] = best_align[4]; trans[0][2] = best_align[8]; trans[0][3] = tr.x;
  trans[1][0] = best_align[1]; trans[1][1] = best_align[5]; trans[1][2] = best_align[9]; trans[1][3] = tr.y;
  trans[2][0] = best_align[2]; trans[2][1] = best_align[6]; trans[2][2] = best_align[10];trans[2][3] = tr.z;
  trans[3][0] = 0;             trans[3][1] = 0;             trans[3][2] = 0;	           trans[3][3] = 1;
  cout << "MC: " << mc << endl;
 
  double thetaX, thetaY, thetaZ;
  thetaY = asin( trans[0][2] );
  if( thetaY < M_PI/2.0 )
    {
      if( thetaY > -M_PI/2.0 )
	{
	  thetaX = atan2( -trans[1][2], trans[2][2] );
	  thetaZ = atan2( -trans[0][1], trans[0][0] );
	}
      else
	{
	  // not a unique solution
	  thetaX = -atan2( trans[1][0], trans[1][1] );
	  thetaZ = 0;
	}
    }
  else
    {
      // not a unique solution
      thetaX = atan2( trans[1][0], trans[1][1] );
      thetaZ = 0;
    }
  cout << "ThetaX: " << thetaX * 180.0 / M_PI << ", ThetaY: " << thetaY * 180.0 / M_PI << ", ThetaZ: " << thetaZ * 180.0 / M_PI << endl;
 
#define histtmax 20
#define histtstep 0.5
#define histcount 40   // int histcount = histtmax / histtstep;
  
  ostringstream stroutt;
  stroutt << "histogram_t-" << map1->scanid << "-" << map2->scanid << "_" << map1->width << "x" << map1->height;
  ofstream outt(stroutt.str().c_str());
  int histogram_t[histcount];
  for (int i = 0 ; i < histcount ; i++)
    {
      histogram_t[i] = 0;
    }
  for (int i = 0 ; i < countm ; i++)
    {
      if (!hasCoordsA[i]) continue;
      if (!hasCoordsB[i]) continue;
      Coord btrans = tr + transformCoord(coordsB[i], best_align);
      double errori = (btrans - coordsA[i]).abs();
      int ind = (int) (errori / histtstep);
      if ( ind > histcount - 1)
        {
	  continue;
	}
      histogram_t[ind] = histogram_t[ind] + 1;
    }
  for (int i = 0 ; i < histcount ; i++)
    {
      outt << (double) i * histtstep << " " << histogram_t[i] << endl;
    }
  outt.close();
  
#define histrmax 180
#define histrstep 2
#define histrcount 90   // int histrcount = histrmax / histrstep;

  ostringstream stroutr;
  stroutr << "histogram_r-" << map1->scanid << "-" << map2->scanid << "_" << map1->width << "x" << map1->height;
  ofstream outr(stroutr.str().c_str());
  int histogram_r[histrcount];
  for (int i = 0 ; i < histrcount ; i++)
    {
      histogram_r[i] = 0;
    }
  for (int i = 0 ; i < countm ; i++)
    {
      if (!hasCoordsA[i]) continue;
      if (!hasCoordsB[i]) continue;
      Coord btrans = tr + transformCoord(coordsB[i], best_align);
      double errori = (btrans - coordsA[i]).abs();
      
      if (errori < t)
        {
	  Coord a = tr;
	  Coord b = Coord(0,0,0);
	  Coord c = coordsA[i];
	  
	  double ca = (c-a).abs();
	  double ab = (a-b).abs();
	  double bc = (b-c).abs();
	  double angle = acos(( - ab*ab + ca*ca + bc*bc)/2.0/ca/bc);
	  angle = angle / 2 / M_PI * 360.0;
	  angle = (angle < 0) ? -angle : angle;
	  int ind = (int) (angle / histrstep);
	  histogram_r[ind]++;
	}
    }
  for (int i = 0 ; i < histrcount ; i++) 
    {
      outr << (double) i * histrstep << " " << histogram_r[i] << endl;
    }
  outr.close();
  return true;
}
