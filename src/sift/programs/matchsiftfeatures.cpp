
/**
 *
 *	@author Darko Makreshanski
 *	this file is a modified version of AutoPano.c from the autopano-sift-C library
 */


/* autopano-sift, Automatic panorama image creation
 * Copyright (C) 2004 -- Sebastian Nowozin
 *
 * This program is free software released under the GNU General Public
 * License, which is included in this software package (doc/LICENSE).
 */

/* GenerateKeys.cs
 *
 * SIFT feature detector keypoint file generator
 *
 * (C) Copyright 2004 -- Sebastian Nowozin (nowozin@cs.tu-berlin.de)
 *
 * "This software is provided for non-commercial use only. The University of
 * British Columbia has applied for a patent on the SIFT algorithm in the
 * United States. Commercial applications of this software may require a
 * license from the University of British Columbia."
 * For more information, see the LICENSE file supplied with the distribution.
 */

#include "PanoramaMap.h"
#include "FeatureSet.h"
#include "FeatureMatchSetGroup.h"
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <assert.h>
#include <ctime>
#include "AutoPanoSift.h"

void usage(int argc, char** argv)
{
	printf("\n");
	printf(
		"USAGE: %s [ -o FILE.matches] file1.keyset file2.keyset [file3.keyset] ...\n",// [ -m MIN_DIM ]\n",
		argv[0]
	);
	printf("\n");
	printf("\n");
	printf("\tOptions:\n");
	printf("\t\t-o FILE.matches\t\tserialize the FeatureMatchSetGroup class\n");
	printf("\t\t-m <maxmatches>\t\tfilter and return the best <maxmatches> matches\n");
	printf("\t\tfile.keyxml\t\tinput file. The xml file output returned by generatesiftfeatures -x\n");
//	printf("\t\t-m MIN_DIM\t\tminDim: (optional) Downscale resolution. The image is repeatedly halfed in size until both width and height are below 'minDim'.\n");
	printf("\n");
//	printf("\tExample:\n");
//	printf("\t\t%s -j -s 1000x300 -s 500x150 example.file\n");
//	printf("\n");

	exit(1);
//	fprintf()
}

Feature getFeatureFromKeypointN(KeypointN* p)
{
	vector<int> desc;
	for (int d = 0 ; d < p->dim ; d++) {
		desc.push_back(p->descriptor[d]);
	}
	Feature feat(p->x, p->y, p->scale, p->orientation, desc);
	return feat;
}

FeatureMatchSetGroup *getFeatureMatchSetGroup(ArrayList* matchsets)
{
	FeatureMatchSetGroup *res = new FeatureMatchSetGroup();
	int count = ArrayList_Count(matchsets);
	for (int i = 0 ; i < count ; i++) {
		MatchSet* ms = (MatchSet*) ArrayList_GetItem(matchsets, i);
		FeatureMatchSet set;
		set.firstscan = ms->file1;
		set.secondscan = ms->file2;
		ArrayList* matches = ms->matches;
		int mcount = ArrayList_Count(ms->matches);
		for (int j = 0 ; j < mcount ; j++) {
			Match* m = (Match*) ArrayList_GetItem(matches, j);
			FeatureMatch match(
				getFeatureFromKeypointN(m->kp1),
				getFeatureFromKeypointN(m->kp2),
				m->dist1,
				m->dist2
			);
			set.matches.push_back(match);
		}
		res->matchsets.push_back(set);
	}
	return res;
}

int main (int argc, char* argv[])
{    

	int downRes = 0;
	vector<char*> inputfiles;
	char *output = NULL;	
	
	int maxmatches = 0;
	
	int c;

	opterr = 0;

	while ((c = getopt (argc, argv, "o:m:")) != -1)
		switch (c)
		{
			case 'o':
				output = optarg;
				break;
			case 'm':
				maxmatches = atoi(optarg);
				if (maxmatches < 0) {
					fprintf( stderr, "maxmatches cannot be negative\n");
					usage(argc, argv);
				}
				break;
			case '?':
				if (optopt == 'o' || optopt == 'm')
					fprintf (stderr, "Option -%c requires an argument.\n", optopt);
 				else
					fprintf (stderr,
						"Unknown option character `%c'\n",
						optopt);
				return 1;
			case ':':
			default:
				usage(argc, argv);
		}
	

	if (output == NULL) {
		printf("You must provide an output file\n");
		usage(argc, argv);
	}

	if (optind > argc - 2) {
		printf("%d %d\n", optind, argc);
		printf("Too few input files. At least two are required.\n");
		usage(argc, argv);
	}

	ArrayList* keyfiles = ArrayList_new0(NULL);

	for (int index = optind; index < argc; index++) {
		ArrayList_AddItem(keyfiles, argv[index]);
	}
	
	MultiMatch* mm = MultiMatch_new0 ();

	WriteLine ("Loading keyfiles");
	MultiMatch_LoadKeysets (mm, keyfiles);

	WriteLine ("\nMatching...");
	clock_t start = clock();
	ArrayList* msList = MultiMatch_LocateMatchSets (mm, 3, maxmatches,
							false, false);
	clock_t end = clock();
	double time = ((double)((int) end - (int) start)) / CLOCKS_PER_SEC;

	cout << "Matched in time: " << time << endl;	
							
	FeatureMatchSetGroup *msg = getFeatureMatchSetGroup(msList);

	msg->serialize(output);
	
	delete msg;
	ArrayList_delete(keyfiles);
	MultiMatch_delete(mm);

	return 0;
}

