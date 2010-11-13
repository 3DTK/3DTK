///*
// * DataMap.cpp
// *
// *  Created on: Feb 14, 2010
// *      Author: darko
// */

//#include "DataMap.h"
//#include <string>
//#include <iostream>
//#include <fstream>
//#include <float.h>

//using namespace std;

//DataMap::DataMap(DataMapSourceType stype, std::string source, int resX, int resY) {

//	this->resX = resX;
//	this->resY = resY;

//	//allocate disk space
//	data = new PMData_Point*[resX];
//	for (int x = 0; x < resX; ++x) {
//		data[x] = new PMData_Point[resY];
//	}

////	float mina, minb, mind, minr, maxa, maxb, maxd, maxr;
//	mina = minb = mind = minr = FLT_MAX;
//	maxa = maxb = maxd = maxr = FLT_MIN;

//	try {

//		switch (stype) {
//		case RIEGL_FILE_SPHERICAL:

//			ifstream file;
//			file.open(source.c_str(), ios::in);
//			if (!file.is_open()) {
//				throw FILE_NOT_FOUND;
//			}

//			long lines;
//			file >> lines;

//			float n;

//			float ta, tb, td, tr;

//			for (long l = 0; l < lines; ++l) {
//				file >> n >> n >> n >> td >> tr >> ta >> tb;
//				if (td < mind) mind = td;
//				if (ta < mina) mina = ta;
//				if (tb < minb) minb = tb;
//				if (tr < minr) minr = tr;
//				if (td > maxd) maxd = td;
//				if (ta > maxa) maxa = ta;
//				if (tb > maxb) maxb = tb;
//				if (tr > maxr) maxr = tr;
//			}

//			cout << mind << " " << mina << " " << minb << " " << minr << endl;
//			cout << maxd << " " << maxa << " " << maxb << " " << maxr << endl;

//			break;
//		}

//	} catch (void* e) {
//		cerr << "Error: " << e << endl;
//	}

//}

//void DataMap::findMinMax(DataMapSourceType stype, string source) {
//	switch(stype) {
//	case RIEGL_FILE_SPHERICAL:
//		break;
//	}
//}


//DataMap::~DataMap() {

//	for (int x = 0; x < resX; ++x) {
//		delete [] data[x];
//	}
//	delete [] data;

//}
