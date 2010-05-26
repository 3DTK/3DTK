/*
 * Reader_RIEGL.cpp
 *
 *  Created on: Feb 20, 2010
 *      Author: darko
 */

#include "Reader_RIEGL.h"

#include "PolarPointCloud.h"
//#include "PolarPoint.h"

#include <fstream>
#include <iostream>
#include <stdlib.h>

#define M_PI 3.14159265

using namespace std;

Reader_RIEGL::Reader_RIEGL() {
	// TODO Auto-generated constructor stub

}

Reader_RIEGL::~Reader_RIEGL() {
	// TODO Auto-generated destructor stub
}

PolarPointCloud Reader_RIEGL::readPolarPointCloud(string scanid, string filename) {
	return Reader_RIEGL::readPolarPointCloud(scanid, filename, 1);
}

PolarPointCloud Reader_RIEGL::readPolarPointCloud(string scanid, string filename, int countevery)
{
//	cout << "YES\n";
	if (countevery < 1) {
		cerr << "Invalid entry for 'countevery' at Reader_RIEGL::readPolarPointCloud" << endl;
		exit(-1);
	}

	ifstream file;
	file.open(filename.c_str(), ios::in);
	if (!file.is_open()) {
		throw FILE_NOT_FOUND;
	}

	long lines;
	file >> lines;

	long pointcount = lines / countevery;

//	pointcount = 16164873;
//	pointcount = 10000000;

//	vector<PolarPoint>* data = new vector<PolarPoint>(pointcount);
	vector<PolarPoint>* data = new vector<PolarPoint>();

	char nullbuf[1024];

	try {
		float n;

		float ta, tb, td, tr;
		double mind = 1000000000;
		double maxd = -1000000000;
		double minr = 1000000000;
		double maxr = -1000000000;
		double minb = 1000000000;
		double maxb = -1000000000;
		double mina = 1000000000;
		double maxa = -1000000000;


		for (long l = 0; l < lines; ++l) {

			if (l % countevery == 0) {
				file.getline(nullbuf, 1024);
				sscanf(nullbuf, "%f %f %f %f %f %f %f", &n, &n, &n, &td, &tb, &ta, &tr);
//				file >> n >> n >> n >> td >> tb >> ta >> tr;
				ta = 360.0 - ta;
				ta = ta * 2.0 * M_PI / 360.0;
				tb -= 90;
				tb *= -1;
				tb *= 2.0 * M_PI / 360.0;

				if (tr > maxr) maxr = tr;
				if (tr < minr) minr = tr;

				tr += 32;
				tr /= 64;

				tr -= 0.2;
				tr /= 0.3;

//				tr *= 3.0;
//				tr -= 0.25;
//				tr *= 4.0/3.0;

				if (tr < 0) tr = 0;
				if (tr > 1) tr = 1;

//				tr = (tr - 30) / 130.0;


//				(*data)[l / countevery] = PolarPoint(ta, tb, td, tr);
				PolarPoint pp;
				pp.a = ta;
				pp.b = tb;
				pp.d = td;
				pp.r = tr;
				data->push_back(pp);
//				(*data)[l / countevery].a = ta;
//				(*data)[l / countevery].b = tb;
//				(*data)[l / countevery].d = td;
//				(*data)[l / countevery].r = tr;
				
//				cout << nullbuf << endl;
//				cout << n << endl;
//				cout << ta << endl;
//				cout << tb << endl;
//				cout << td << endl;
//				cout << tr << endl;

				if (td < mind) mind = td;
				if (td > maxd) maxd = td;
				if (ta < mina) mina = ta;
				if (ta > maxa) maxa = ta;
				if (tb < minb) minb = tb;
				if (tb > maxb) maxb = tb;

			} else {
				file.getline(nullbuf, 1024);
			}

		}

		cout << minr << " refl " << maxr << endl;
		cout << mind << " dist " << maxd << endl;
		cout << minb / 2 / M_PI * 360 << " b " << maxb / 2 / M_PI * 360 << endl;

	} catch(void* e) {
		throw e;
//		throw INVALID_DATA;
	}

	
	file.close();


	return PolarPointCloud(scanid, data, pointcount, 0, 360, -40, 60);

}

PointCloud Reader_RIEGL::readPointCloud(string filename) {
	return Reader_RIEGL::readPointCloud(filename, 1);
}

PointCloud Reader_RIEGL::readPointCloud(string filename, int countevery)
{

//	if (countevery < 1) {
//		cerr << "Invalid entry for 'countevery' at Reader_RIEGL::readPolarPointCloud" << endl;
//		exit(-1);
//	}

//	ifstream file;
//	file.open(filename.c_str(), ios::in);
//	if (!file.is_open()) {
//		throw FILE_NOT_FOUND;
//	}

//	long lines;
//	file >> lines;

//	long pointcount = lines / countevery;

//	Point* data;
//	data = new Point[pointcount];

//	char nullbuf[1024];

//	try {
//		float n;

////		double ta, tb, td, tr;
////		double mind = 1000000000;
////		double maxd = -1000000000;

//		double tx, ty, tz, tr;
//		double maxr = -1000000000;
//		double minr = 1000000000;

//		for (long l = 0; l < lines; ++l) {

//			if (l % countevery == 0) {
//				file >> tx >> ty >> tz >> n >> n >> n >> tr;

////				tr = (tr - 30) / 130.0;

//				tr += 32;
//				tr /= 64;

//				tr -= 0.2; //histogram spreading
//				tr /= 0.3; //histogram spreading

//				if (tr < 0) tr = 0;
//				if (tr > 1) tr = 1;

//				data[l / countevery] = Point(tx, tz, ty, tr);
//			} else {
//				file.getline(nullbuf, 1024);
//			}

////			if (td < mind) mind = td;
//	//		if (ta < mina) mina = ta;
//	//		if (tb < minb) minb = tb;
////			if (td > maxd) maxd = td;
//	//		if (ta > maxa) maxa = ta;
//	//		if (tb > maxb) maxb = tb;
//			if (tr > maxr) maxr = tr;
//			if (tr < minr) minr = tr;
//		}

//		cout << minr << endl << maxr << endl;

//	} catch(void* e) {
//		throw e;
////		throw INVALID_DATA;
//	}

//	return PointCloud(data, pointcount);

}

