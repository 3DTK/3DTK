

#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <string>
#include <vector>

#include "FeatureMatchSetGroup.h"
#include "PanoramaMap.h"
#include "opengl_objects/PanoramaMap_gl.h"
#include "opengl_objects/FeatureMatchSet_gl.h"

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
		"USAGE: %s -m FILE.matches -i MAP1.map -i MAP2.map\n",
		argv[0]
	);
	printf("\n");
	printf("\n");
	printf("\tOptions:\n");
	printf("\t\t-m FILE.matches\t\tinput .matches file (serialization of a FeatureMatchSetGroup class object)\n");
	printf("\t\t-i FILE.map\t\tinput .map file (serialization of a PanoramaMap class object)\n");
	printf("\n");
//	printf("\tExample:\n");
//	printf("\t\t%s -i example.map\n", argv[0]);
//	printf("\n");

	exit(1);
//	fprintf()
}

int main(int argc, char** argv)
{
	
	char *input = NULL;

	char *first = NULL;
	char *second = NULL;

	opterr = 0;
	int c;

	while ((c = getopt (argc, argv, "m:i:")) != -1)
		switch (c)
		{
			case 'm':
				input = optarg;
				break;
			case 'i':
				if (first == NULL) {
					first = optarg;
				} else if (second == NULL) {
					second = optarg;
				} else {
					fprintf(stderr, "Only two maps can be included\n");
					usage(argc, argv);
				}
				break;
			case '?':
				if (optopt == 'm' || optopt == 'i')
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

	if (optind != argc || input == NULL || first == NULL || second == NULL) {
		usage(argc, argv);
	}
	printf("%s\n", input);
	FeatureMatchSetGroup group(input);
	PanoramaMap map1(first);
	PanoramaMap map2(second);

	FeatureMatchSet *set = NULL;
	bool reversed;

	list<FeatureMatchSet>::iterator it;
	for (it = group.matchsets.begin() ; it != group.matchsets.end() ; it++) {
		if (map1.scanid == (*it).firstscan && map2.scanid == (*it).secondscan) {
			reversed = false;
			set = &(*it);
		}
		if (map2.scanid == (*it).firstscan && map1.scanid == (*it).secondscan) {
			reversed = true;
			set = &(*it);
		}
	}
	
	if (set == NULL) {
		printf("There is no such pair of maps in the given FeatureMatchSetGroup input file\n");
		exit(1);
	}

	PanoramaMap_gl glmap1(&map1);
	PanoramaMap_gl glmap2(&map2);
	
	double trx = map1.width * PanoramaMap_gl::scale;
	glmap2.TrX = trx;
	glmap2.updateMultMat();

	FeatureMatchSet_gl fmsgl1(set, &map1, (reversed) ? FMS_second : FMS_first);
	FeatureMatchSet_gl fmsgl2(set, &map2, (reversed) ? FMS_first : FMS_second);
	fmsgl2.TrX = trx;	
	fmsgl2.updateMultMat();
	
	GL::addObject(&glmap1);
	GL::addObject(&glmap2);
	GL::addObject(&fmsgl1);
	GL::addObject(&fmsgl2);

	GL::execute();

	return 0;
	
}
