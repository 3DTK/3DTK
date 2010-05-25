

#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <string>
#include <vector>
#include <iostream>
#include <ctime>

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
		"USAGE: %s [-m] [-j] -s <WIDTHxHEIGHT> [-s <WIDTHxHEIGHT> ...] FILE1.ppc [FILE2.ppc] ...\n",
		argv[0]
	);
	printf("\n");
	printf("\n");
	printf("\tOptions:\n");
	printf("\t\t-m \t\t\twrite to a .map file (serialization of PanoramaMap class):\n");
	printf("\t\t-j \t\t\twrite an image to .jpg file:\n");
	printf("\t\t-s <WxH>\t\tsize of output in widthxheight\n");
	printf("\t\t\t\t\tinput files should be serialization of PolarPointCloud class returned by readscan\n");
	printf("\n");
	printf("\tExample:\n");
	printf("\t\t%s -j -s 1000x300 -s 500x150 example.file\n");
	printf("\n");

	exit(1);
//	fprintf()
}

int main(int argc, char** argv)
{
	
	char *reader = NULL;

	bool writeMaps = false;
	bool writeJpegs = false;

	char *input = NULL;
	char output[1000];

	vector<struct size> sizes;

	int c;

	opterr = 0;

	int width;
	int height;

	while ((c = getopt (argc, argv, "r:mjs:")) != -1)
		switch (c)
		{
			case 'r':
				reader = optarg;
				break;
			case 'm':
				writeMaps = true;
				break;
			case 'j':
				writeJpegs = true;
				break;
			case 's':
				char *pch;
				pch = strtok(optarg, "x");
				width = atoi(pch);
				if (width <= 0 || pch == NULL) {printf("Invalid size\n"); usage(argc, argv);}
				pch = strtok(NULL, "x");
				if (pch == NULL) {printf("Invalid size\n"); usage(argc, argv);}
				height = atoi(pch);
				if (height <= 0) {printf("Invalid size\n"); usage(argc, argv);}
				size sz;
				sz.width = width;
				sz.height = height;
				sizes.push_back(sz);
				break;
			case '?':
				if (optopt == 'r' || optopt == 'w' || optopt == 'h')
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

	if (optind == argc) {
		usage(argc, argv);
	}
	
	if (reader == NULL ||
		
		!(
			strcmp(reader, "RIEGL") == 0
		)
		
	) {
		reader = "RIEGL";
	}

	if (!writeMaps && !writeJpegs) {
		printf("One of the options -m or -j must be included\n");
		usage(argc, argv);
	}

	clock_t start, end;

	vector<struct size>::iterator it;
	char outputmap[1000];
	char outputjpg[1000];
	for (int index = optind ; index < argc ; index++ )
	{
		input = argv[index];
		printf("Reading %s\n", input);
		PolarPointCloud polarcloud(input);
		for (it = sizes.begin() ; it != sizes.end() ; it++ ) {
			cout << "Creating Map " << it->width << " " << it->height << endl;
			start = clock();
			PanoramaMap map(&polarcloud, it->width, it->height);
			end = clock();
			double time = ((double)((int) end - (int) start)) / CLOCKS_PER_SEC;
			cout << "Finished creating map with time: " << time << endl;
			if (writeMaps) {
				sprintf(outputmap, "%s_%dx%d.map", input, it->width, it->height);
				map.serialize(outputmap);
				printf("Wrote map to: %s\n", outputmap);
			}
			if (writeJpegs) {
				sprintf(outputjpg, "%s_%dx%d.jpg", input, it->width, it->height);
				map.toJpeg(outputjpg);				
				printf("Wrote image to: %s\n", outputjpg);
			}
		}
	}

	return 0;
	
}
