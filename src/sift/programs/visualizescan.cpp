

#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <string>
#include <vector>

#include "PolarPointCloud.h"
#include "PointCloud.h"
#include "opengl_objects/PolarPointCloud_gl.h"
#include "opengl_objects/PointCloud_gl.h"
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
		"USAGE: %s [-m <mode> ] -i FILE\n",
		argv[0]
	);
	printf("\n");
	printf("\n");
	printf("\tOptions:\n");
	printf("\t\t-m <2D/3D>\t\ttype of representation 2D/3D (default 2D)\n");
	printf("\t\t-i FILE\t\tinput .ppc file (serialization of a PolarPointCloud class object returned by readscan)\n");
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

	int mode = 0;

	opterr = 0;
	int c;

	while ((c = getopt (argc, argv, "i:m:")) != -1)
		switch (c)
		{
			case 'i':
				input = optarg;
				break;
			case 'm':
				if (strcmp(optarg, "3D") == 0) {
					mode = 1;
				} else
				if (strcmp(optarg, "2D") == 0) {
					mode = 0;
				} else {
					fprintf(stderr, "Invalid mode\n");
					usage(argc, argv);
				}
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
 	PolarPointCloud ppc(input);
 	
 	if (mode == 0) {
		PolarPointCloud_gl glppc(&ppc);
		GL::addObject(&glppc);
		GL::execute();
	} else {
		PointCloud pc(&ppc);
		PointCloud_gl glpc(&pc);
		GL::addObject(&glpc);
		GL::execute();
	}

	return 0;
	
}
