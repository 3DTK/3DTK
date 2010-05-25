

#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <string>
#include <vector>
#include <iostream>
#include <fstream>

#include "Reader_RIEGL.h"
#include "PanoramaMap.h"

using namespace std;

struct size {
	int width;
	int height;
};

void usage(int argc, char** argv)
{
	printf("\n");
	printf(
		"USAGE: %s [ -r RIEGL ] [ -d SCAN_ID ]-o OUTPUT.ppc INPUT_FILE\n",
		argv[0]
	);
	printf("\n");
	printf("\n");
	printf("\tOptions:\n");
	printf("\t\t-r RIEGL\t\ttype of scans (for now just RIEGL is supported)\n");
	printf("\t\t-d scanid\t\tscan id, default is the scan filename\n");
	printf("\t\t-o \t\t\twrite to a .ppc file (serialization of PolarPointCloud class):\n");
	printf("\n");
//	printf("\tExample:\n");
//	printf("\t\t%s -j -s 1000x300 -s 500x150 example.file\n");
	printf("\n");

	exit(1);
//	fprintf()
}

int main(int argc, char** argv)
{
	
	char *reader = NULL;

	char *input = NULL;
	char *output = NULL;

	char *scanid = NULL;

	int c;

	opterr = 0;

	int width;
	int height;

	while ((c = getopt (argc, argv, "o:d:")) != -1)
		switch (c)
		{
			case 'o':
				output = optarg;
				break;
			case 'd':
				scanid = optarg;
				break;
			case '?':
				if (
						optopt == 'o' 
							|| 
						optopt == 'd' 
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

	if (output == NULL) {
		usage(argc, argv);
	}
	
	if (optind != argc - 1) {
		usage(argc, argv);
	}
	
	input = argv[optind];
	
	if (reader == NULL ||
		
		!(
			strcmp(reader, "RIEGL") == 0
		)
		
	) {
		reader = "RIEGL";
	}

	string scid;
	if (scanid == NULL) {
		scid = input;
	} else {
		scid = scanid;
	}
	PolarPointCloud polarcloud = Reader_RIEGL::readPolarPointCloud(scid, input, 1);
	polarcloud.serialize(output);

	int ind[100];
	for (int i = 0 ; i < 100 ; i++) {
		ind[i] = 0;
	}
	for (int i = 0 ; i < polarcloud.length ; i++) {
		int index = (int)((*polarcloud.data)[i].r * 100.0);
		if (index < 0) index = 0;
		if (index > 99) index == 99;
		ind[index]++;
	}

	ofstream of("plot");
	for (int i = 0 ; i < 100 ; i++) {
		of << i << " " << ind[i] << endl;
	}
	of.close();
	
	return 0;
	
}
