/*
 * PanoramaMap.cpp
 *
 *  Created on: Feb 20, 2010
 *      Author: darko
 */

#include "PanoramaMap.h"
#include <iostream>
#include <fstream>
#include <new>

#include <vector>

#include <vigra/imageinfo.hxx>
#include <vigra/impex.hxx>
#include <math.h>

using namespace std;

#define MAX_ANGLE  60.0
#define MIN_ANGLE  -40.0
//PanoramaMap::PanoramaMap() {
//	// TODO Auto-generated constructor stub
//
//}

PanoramaMap::PanoramaMap()
{

//	int nwidth = 1440;
//	int nheight = 400;

////	vector< vector<int> > index(nwidth, vector<int>(nheight, 0));
//	data = vector< vector<SuperPixel> >(nwidth, vector<SuperPixel>(nheight));

//	width = nwidth;
//	height = nheight;

}


PanoramaMap::PanoramaMap(PolarPointCloud* cloud, int nwidth, int nheight)
{

	scanid = cloud->scanid;

	vector< vector<int> > index(nwidth, vector<int>(nheight, 0));


//	for (int i = 0 ; i < nwidth ; i++ ) {
//		data.push_back(vector<SuperPixel>());
//		for (int j = 0 ; j < nheight ; j++) {
//			data[i].push_back(SuperPixel());
//		}
//	}


	data = new SuperPixel*[nwidth];
	for (int i = 0 ; i < nwidth ; i++) {
		data[i] = new SuperPixel[nheight];
		for (int j = 0 ; j < nheight ; j++) {
			data[i][j].value = 0;
			data[i][j].meta = 0;
		}
	}	
//	data = vector< vector<SuperPixel> >(nwidth, vector<SuperPixel>(nheight));


	long length = cloud->getLength();
	const vector<PolarPoint> *pdata = cloud->getData();
	double xmaxt = (double) nwidth / 2 / M_PI;
	int xmax = nwidth - 1;
	double ymaxt = (double) nheight / ((MAX_ANGLE - MIN_ANGLE) / 360 * 2 * M_PI);
	double ylow = (0 - MIN_ANGLE) / 360 * 2 * M_PI;
	int ymax = nheight - 1;

	for (int i = 0 ; i < length ; i++) {
		int x = (int) ( xmaxt * (*pdata)[i].a);
		if (x < 0) x = 0;
		if (x > xmax) x = xmax;
		int y = (int) ( ymaxt * ((*pdata)[i].b + ylow) );
		if (y < 0) y = 0;
		if (y > ymax) y = ymax;
		index[x][y] = index[x][y] + 1;
		data[x][y].value += (*pdata)[i].r;
		data[x][y].meta += (*pdata)[i].d;
	}

	int ind;
	for (int i = 0 ; i < nwidth ; i++) {
		for (int j = 0 ; j < nheight ; j++) {
			ind = index[i][j];
			if (ind != 0) {
//				cout << data[i][j].value << endl;
				data[i][j].value /= ind;
//				cout << data[i][j].value << endl;
				data[i][j].meta /= ind;
			} else {
				data[i][j].value = 0;
				data[i][j].meta = 0;
			}
		}
	}
	
	width = nwidth;
	height = nheight;
	
	mina = cloud->mina;
	maxa = cloud->maxa;
	minb = cloud->minb;
	maxb = cloud->maxb;

}

PanoramaMap::~PanoramaMap() {

//	for (int i = 0 ; i < width ; i++) {
//		cout << i << endl;
//		for (int j = 0 ; j < height ; j++) {
//			if (i > 726) cout << j << " ";
//			if (i > 726 && j == 399) {
//				cout << data[i].back().value << " " << data[i].back().meta << endl;				
//			}
//			data[i].pop_back();
//		}
//		data.pop_back();
//	}

	for (int i = 0 ; i < width; i++) {
		delete [] data[i];
	}
	delete [] data;
//	
}



void PanoramaMap::toJpeg(std::string filename)
{
	vigra::ImageExportInfo info(filename.c_str());
	info.setCompression("90");

	vigra::BRGBImage img(width, height);

	for (int x = 0 ; x < width ; x++) {
		for (int y = 0 ; y < height ; y++) {
//			img(x,height - y - 1) = vigra::RGBValue<unsigned char>(data[x][y].value * 255);
			img(x,height - y - 1) = vigra::RGBValue<unsigned char>((1-log10(data[x][y].meta)/2.6) * 255);
		}
	}

	vigra::exportImage(vigra::srcImageRange(img), info);
}

void PanoramaMap::serialize(const char* filename)
{
	string str(filename);
	cout << filename << endl;
	ofstream out(filename, ios::binary);
	int stringsize = scanid.size();
	out.write((char*) &stringsize, sizeof(stringsize));
	for (int i = 0 ; i < stringsize ; i++) {
		char s = scanid[i];
		out.write((char*)&s, sizeof(s));
	}	
	
	out.write((char *) &width, sizeof(width));
	out.write((char *) &height, sizeof(height));
	for (int x = 0 ; x < width ; x++) {
		for (int y = 0 ; y < height ; y++) {
			out.write((char *) &(data[x][y]), sizeof(data[x][y]));
		}
	}

	out.write((char*) &mina, sizeof(mina));
	out.write((char*) &maxa, sizeof(maxa));
	out.write((char*) &minb, sizeof(minb));
	out.write((char*) &maxb, sizeof(maxb));

	out.close();
}

PanoramaMap::PanoramaMap(const char* filename)
{
	ifstream in(filename, ios::binary);
	if (!in.good()) {cerr << "File not found" << endl; throw 1; }

	int stringsize;
	in.read((char*) &stringsize, sizeof(stringsize));
	scanid = "";
	for (int i = 0 ; i < stringsize ; i++) {
		char s;
		in.read((char*)&s, sizeof(s));
		scanid += s;
	}

	in.read((char*)&width, sizeof(width));
	in.read((char*)&height, sizeof(height));
	
	data = new SuperPixel*[width];
	for (int i = 0 ; i < width ; i++) {
		data[i] = new SuperPixel[height];
	}	
//	data = vector< vector<SuperPixel> >(width, vector<SuperPixel>(height));
	
	for (int x = 0 ; x < width ; x++) {
		for (int y = 0 ; y < height ; y++) {
			in.read((char *) &(data[x][y]), sizeof(data[x][y]));
		}
	}

	in.read((char*) &mina, sizeof(mina));
	in.read((char*) &maxa, sizeof(maxa));
	in.read((char*) &minb, sizeof(minb));
	in.read((char*) &maxb, sizeof(maxb));

	in.close();
}
