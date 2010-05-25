

#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <string>
#include <vector>

#include <fstream>

#include "PolarPointCloud.h"
#include "PointCloud.h"
#include "opengl_objects/PointCloud_gl.h"

#include "opengl-framework/GL.h"

using namespace std;

struct size {
	int width;
	int height;
};
 
void usage(int argc, char** argv)
{
	printf("\n");
	printf(
		"USAGE: %s [ -m <number>] [ -c ] -p SCAN1.ppc -p SCAN2.ppc [ -p SCAN3.ppc ...] \n",
		argv[0]
	);
	printf("\n");
	printf("\tNote:\n");
	printf("\t\t.dat files of the respective scans must be present (output of registerscans)");
	printf("\n");
	printf("\tOptions:\n");
	printf("\t\t-m <number>\t\tRead every <number>-th point from the scans (default 20)\n");
	printf("\t\t-p SCAN.ppc\t\tinput .ppc file serialization of polar point cloud class\n");
	printf("\t\t-c \t\t\tuse random colors for each scan\n");
	printf("\n");
//	printf("\tExample:\n");
//	printf("\t\t%s -i example.map\n", argv[0]);
//	printf("\n");

	exit(1);
//	fprintf()
}

vector<PointCloud*> clouds;
vector<PointCloud_gl*> clouds_gl;

void atExit(void)
{
	cout << "Freeing pointclouds...";
	int size = clouds.size();
	for (int i = 0 ; i < size ; i++) {
		delete (clouds[i]);
		delete (clouds_gl[i]);
	}
	cout << "OK\n";
}

int main(int argc, char** argv)
{
	
	int m = 10;
	vector<string> scans;

	bool randc = false;
	bool rainc = false;

	opterr = 0;
	int c;

	while ((c = getopt (argc, argv, "m:p:cr")) != -1)
		switch (c)
		{
			case 'c':
				randc = true;
				rainc = false;
				break;
			case 'r':
				rainc = true;
				randc = false;
				break;
			case 'm':
				m = atoi(optarg);
				if (m <= 0) {
					cerr << "Invalid number for -m !\n";
					usage(argc, argv);
				}
				break;
			case 'p':
				scans.push_back(optarg);
				break;
			case '?':
				if (optopt == 'm' || optopt == 'p')
					fprintf (stderr, "Option -%c requires an argument.\n", optopt);
 				else
					fprintf (stderr,
						"Unknown option character `%c'.\n",
						optopt);
				return 1;
			case ':':
			default:
				usage(argc, argv);
		}

	if (scans.size() < 2) {
		cerr << "At least two scans are required\n";
		usage(argc, argv);	
	}

	for (int i = 0 ; i < scans.size() ; i++) {
		cout << "Reading ppc: " << scans[i] << endl;
		PolarPointCloud ppc( scans[i].c_str(), m);
		cout << "Converting to pointcloud...\n";
		clouds.push_back(new PointCloud(&ppc));
		string s = ppc.scanid + ".dat";
		cout << "Reading orientation: " << s << endl;
		ifstream f(s.c_str());
		string scanid;
		f >> scanid;
		double tr[4][4];
		for (int x = 0 ; x < 4 ; x++) {
			for (int y = 0 ; y < 4 ; y++) {
				f >> tr[x][y];
			}
		}
		f.close();
		PointCloud_gl *pcgl = new PointCloud_gl((clouds[i]));
		pcgl->setMultMat(tr);
		
		if (randc) {

//			Coord c;

//			float v = rand() % 70 + 15;
//		
//			if (v < 16.66) {
//				c.z = v / 16.66;
//			} else if (v < 33.33) {
//				c.z = 1;
//				c.y = (v - 16.66) / 16.66;
//			} else if (v < 50) {
//				c.z = (50 - v) / 16.66;
//				c.y = 1;
//			} else if (v < 66.66) {
//				c.x = (v - 50) / 16.66;
//				c.y = 1;
//			} else if (v < 83.33) {
//				c.y = (83.33 - v) / 16.66;
//				c.x = 1;
//			} else {
//				c.x = (100 - v) / 16.66;
//			}

//			pcgl->setColor(c);


			float ratio = (float) (i + 1) / (float) scans.size();
			float rat1 = (float) ((int) (sqrt(ratio) * 100000000.0) % 5000) / 5000.0;
			float rat2 = (float) ((int) (sqrt(ratio) * 10000000.0) % 5000) / 5000.0;
			float rat3 = (float) ((int) (sqrt(ratio) * 1000000.0) % 5000) / 5000.0;
			rat1/=3;
			rat2/=3;
			rat3/=3;
			rat1+=0.66;
			rat2+=0.66;
			rat3+=0.66;
			pcgl->setColor(Coord(rat1, rat2, rat3));


		} else if (rainc) {
			pcgl->rainbowcolor = true;
		}
		
		clouds_gl.push_back(pcgl);
		GL::addObject(pcgl);
	}
	
	atexit(atExit);
	GL::execute();

	return 0;
	
}
