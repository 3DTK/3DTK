/*
 * PolarPointCloud.cpp
 *
 *  Created on: Feb 20, 2010
 *      Author: darko
 */

#include "PolarPointCloud.h"
#include <iostream>
#include <math.h>
#include <fstream>

using namespace std;

PolarPointCloud::PolarPointCloud(string scanid, vector<PolarPoint> *_data, long length, double mina, double maxa, double minb, double maxb)
{
	this->scanid = scanid;
	this->data = _data;;
	this->length = length;
	this->mina = mina;
	this->maxa = maxa;
	this->minb = minb;
	this->maxb = maxb;
}


PolarPointCloud::PolarPointCloud(PointC *tdata, long length )
{

	data = new vector<PolarPoint>(length);
	PolarPoint *pdata = new PolarPoint[length];

//	double minb = 100000000;
//	double maxb = -100000000;

	double a,b,d,x,y,z;
	for (int i = 0 ; i < length ; ++i) {
		x = tdata[i].x;
		y = tdata[i].y;
		z = tdata[i].z;
		d = sqrt(x*x + y*y + z*z);
		a = atan2(z,x);
		b = asin(y / d);
		(*data)[i].a = a;
		(*data)[i].b = b;
		(*data)[i].d = d;
		(*data)[i].r = tdata[i].r;

//		if (b < minb) minb = b;
//		if (b > maxb) maxb = b;
	}

//	cout << minb / 2 / M_PI * 360 << " cb " << maxb / 2 / M_PI * 360 << endl;

	this->length = length;

}

//void PolarPointCloud::setData(PolarPoint *data, long length)
//{

//}

long PolarPointCloud::getLength() const
{
	return this->length;
}

const vector<PolarPoint> *PolarPointCloud::getData() 
{
	return data;
}

PolarPointCloud::~PolarPointCloud() {
//	std::cout << "Freeing PPC data\n";
//	delete [] this->data;
//	delete data;
}


void PolarPointCloud::serialize(const char* filename)
{
	ofstream out(filename, ios::binary);

	int stringsize = scanid.size();
	out.write((char*) &stringsize, sizeof(stringsize));
	for (int i = 0 ; i < stringsize ; i++) {
		char s = scanid[i];
		out.write((char*)&s, sizeof(s));
	}

	out.write((char*) &length, sizeof(length));
	for (int i = 0 ; i < length ; i++) {
		double a,b,r,d;
		PolarPoint p = (*data)[i];
		a = p.a;
		b = p.b;
		r = p.r;
		d = p.d;
		out.write((char*) &a, sizeof(a));
		out.write((char*) &b, sizeof(b));
		out.write((char*) &r, sizeof(r));
		out.write((char*) &d, sizeof(d));
	}	
	out.write((char*) &mina, sizeof(mina));
	out.write((char*) &maxa, sizeof(maxa));
	out.write((char*) &minb, sizeof(minb));
	out.write((char*) &maxb, sizeof(maxb));

	out.close();
}

PolarPointCloud::PolarPointCloud(const char* filename, int readevery)
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

	in.read((char*)&length, sizeof(length));
	
	data = new vector<PolarPoint>();
	for (int i = 0 ; i < length ; i++) {
		PolarPoint p;
		double a,b,r,d;
		in.read((char*) &a, sizeof(a));
		in.read((char*) &b, sizeof(b));
		in.read((char*) &r, sizeof(r));
		in.read((char*) &d, sizeof(d));
		p.a = a;
		p.b = b;
		p.r = r;
		p.d = d;
		if (i % readevery == 0) {
			data->push_back(p);
		}
	}	

	length = data->size();

	in.read((char*) &mina, sizeof(mina));
	in.read((char*) &maxa, sizeof(maxa));
	in.read((char*) &minb, sizeof(minb));
	in.read((char*) &maxb, sizeof(maxb));

	in.close();
}
