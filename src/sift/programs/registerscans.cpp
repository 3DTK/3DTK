

#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <string>
#include <vector>
#include <ctime>

#include "FeatureMatchSetGroup.h"
#include "PanoramaMap.h"
#include "Register.h"

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
		"USAGE: %s -m FILE.matches MAP1.map MAP2.map [MAP3.map ...]\n",
		argv[0]
	);
	printf("\n");
	printf("\n");
	printf("\tOptions:\n");
	printf("\t\t-m FILE.matches\t\tinput .matches file (serialization of a FeatureMatchSetGroup class object)\n");
	printf("\t\t-d <int>\t\t minimum number of pairs that should fit the model (default: 2)\n");
	printf("\t\t-t <float>\t\t minimum threshold distance that determines if it is a fit or not (default: 1.0)\n");
	printf("\t\t-i <float>\t\t influence of the numbers of inliers to the error term (default: 0.05)\n");
	printf("\t\t-r <float>\t\t minimum distance between any two points in a triangle (default: 5)\n");
//	printf("\t\t-a <float>\t\t minimum angle in a triangle\n");
	printf("\t\tFILE.map\t\tinput .map file (serialization of a PanoramaMap class object)\n");
	printf("\n");
//	printf("\tExample:\n");
//	printf("\t\t%s -i example.map\n", argv[0]);
//	printf("\n");

	exit(1);
//	fprintf()
}

int main(int argc, char** argv)
{
	
	vector<char*> input;

	int d = 0;
	double t = 0;
	double i = 0;
	double r = 0;
	double a = 0;

	int modemax = 0;

	opterr = 0;
	int c;

	while ((c = getopt (argc, argv, "m:d:t:i:r:a:x:")) != -1)
		switch (c)
		{
			case 'x':
				modemax = atoi(optarg);
				break;
			case 'm':
				input.push_back(optarg);
				break;
			case 'd':
				d = atoi(optarg);
				if (d <= 0) {
					cout << "Invalid integer for d\n";
					usage(argc, argv);
				}
				break;
			case 't':
				t = atof(optarg);
				if (t <= 0) {
					cout << "Invalid integer for t\n";
					usage(argc, argv);
				}
				break;
			case 'i':
				i = atof(optarg);
//				if (i <= 0) {
//					cout << "Invalid integer for i\n";
//					usage(argc, argv);
//				}
				break;
			case 'r':
				r = atof(optarg);
				if (r <= 0) {
					cout << "Invalid integer for r\n";
					usage(argc, argv);
				}
				break;
			case 'a':
				a = atof(optarg);
				if (a <= 0) {
					cout << "Invalid integer for a\n";
					usage(argc, argv);
				}
				break;
			case '?':
				if (
						optopt == 'm'
							||
						optopt == 'd'
							||
						optopt == 't'
							||
						optopt == 'i'
							||
						optopt == 'r'
							||
						optopt == 'a'
						)
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


	if (input.size() == 0) {
		usage(argc, argv);
	}

	FeatureMatchSetGroup group;
	for (int i = 0 ; i < (int) input.size() ; i++) {
		cout << "Reading match set group: " << input[i] << endl;
		FeatureMatchSetGroup group_2(input[i]);
		group.matchsets.merge(group_2.matchsets);
//		for (list<FeatureMatchSet>::iterator it = group_2.matchsets.begin() ; it != group_2.matchsets.end() ; it++) {	
//			cout << it->firstscan << " " << it->secondscan << endl;
//			group.matchsets.push_back(*it);
//		}
	}
	cout << group.matchsets.size() << endl;
	
	vector<PanoramaMap*> maps;
	for (int index = optind; index < argc ; index++) {
		cout << "Reading map: " << argv[index] << endl;
		PanoramaMap *map = new PanoramaMap(argv[index]);
		maps.push_back(map);
	}

	cout << "Checking..\n";
	Register reg(&group, maps);
	if (modemax != 0) reg.mode_maximum = modemax;
	if (d != 0) reg.d = d;
	if (t != 0) reg.t = t;
	if (i != 0) reg.dinfluence = i;
	if (r != 0) reg.mind = r;
	if (a != 0) reg.mina = a / 360.0 * 2 * M_PI;
	
	cout << "Registering..\n";
	clock_t start = clock();
	reg.registerScans();
	clock_t end = clock();
	double time = ((double)((int) end - (int) start)) / CLOCKS_PER_SEC;

	cout << "Time needed for registering: " << time << endl;

	cout << "Freeing data...\n";
	for (int i = 0 ; i < maps.size() ; i++) {
		delete maps[i];
	}

	return 0;
	
}
