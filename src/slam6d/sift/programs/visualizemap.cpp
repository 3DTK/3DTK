

#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <string>
#include <vector>

#include "PanoramaMap.h"
#include "opengl_objects/PanoramaMap_gl.h"
#include "opengl_framework/GL.h"

using namespace std;

struct size {
	int width;
	int height;
};

void usage(int argc, char** argv)
{
	printf("\n");
	printf(
		"USAGE: %s -i FILE\n",
		argv[0]
	);
	printf("\n");
	printf("\n");
	printf("\tOptions:\n");
	printf("\t\t-i FILE\t\tinput .map file (serialization of a PanoramaMap class object)\n");
	printf("\n");
	printf("\tExample:\n");
	printf("\t\t%s -i example.map\n", argv[0]);
	printf("\n");

	exit(1);
//	fprintf()
}

int main(int argc, char** argv)
{
	
	char *input = NULL;

	opterr = 0;
	int c;

	while ((c = getopt (argc, argv, "i:")) != -1)
		switch (c)
		{
			case 'i':
				input = optarg;
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

	if (optind != argc || input == NULL) {
		usage(argc, argv);
	}
	printf("%s\n", input);
 	PanoramaMap map(input);
	PanoramaMap_gl glmap(&map);
	GL::addObject(&glmap);

	GL::execute();

	return 0;
	
}
