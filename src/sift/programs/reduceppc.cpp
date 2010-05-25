

#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <string>
#include <vector>
#include <iostream>

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
		"USAGE: %s -m <number> -o OUTPUT.ppc INPUT.ppc\n",
		argv[0]
	);
	printf("\n");
	printf("\n");
	printf("\tOptions:\n");
	printf("\t\t-m <number>\t\treduce to every <number>-th point\n");
	printf("\t\t-o scanid\t\toutput .ppc\n");
	printf("\n");
//	printf("\tExample:\n");
//	printf("\t\t%s -j -s 1000x300 -s 500x150 example.file\n");
	printf("\n");

	exit(1);
//	fprintf()
}

int main(int argc, char** argv)
{
	
	int m = 0;

	char *input = NULL;
	char *output = NULL;


	int c;

	opterr = 0;

	int width;
	int height;

	while ((c = getopt (argc, argv, "m:o:")) != -1)
		switch (c)
		{
			case 'm':
				m = atoi(optarg);
				if (m <= 0) {
					cerr << "Invalid number\n";
					usage(argc, argv);
				}
				break;
			case 'o':
				output = optarg;
				break;
			case '?':
				if (
						optopt == 'm' 
							|| 
						optopt == 'o' 
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
	
	cout << "Reading ppc (m = " << m << ")from: " << input << endl;
	PolarPointCloud polarcloud(input, m);
	cout << "Writing reduced ppc to: " << output << endl;
	polarcloud.serialize(output);
	
	return 0;
		
}
