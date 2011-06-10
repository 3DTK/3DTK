/**
 * @file matchsiftfeatures.cc
 * @brief this file is a modified version of AutoPano.C from the autopano-sift-C library
 * @author Andreas Nuechter. Jacobs University Bremen gGmbH, Grmany.
 * @author HamidReza Houshiar. Jacobs University Bremen gGmbH, Germany.
 * @author Darko Makreshanski. Jacobs University Bremen gGmbH, Germany.
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

#include "slam6d/sift/library/PanoramaMap.h"
#include "slam6d/sift/library/FeatureSet.h"
#include "slam6d/sift/library/FeatureMatchSetGroup.h"
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <ctime>
#include "autopano-sift-c_modified/AutoPanoSift.h"

#ifndef _MSC_VER
#include <getopt.h>
#else
#include "..\..\..\Visual_Studio_Projects\6DSLAM\6D_SLAM\XGetopt.h"
#endif

/**
 *   usage - explains how to use the program CMD
 */
void usage(int argc, char** argv)
{
  printf("\n");
  printf("USAGE: %s [ -o FILE.matches] file1.keyset file2.keyset [file3.keyset] ...\n",/* [ -m MIN_DIM ]\n",*/argv[0]);
  printf("\n");
  printf("\n");
  printf("\tOptions:\n");
  printf("\t\t-o FILE.matches\t\tserialize the FeatureMatchSetGroup class\n");
  printf("\t\t-m <maxmatches>\t\tfilter and return the best <maxmatches> matches\n");
  printf("\t\tfile.keyxml\t\tinput file. The xml file output returned by generatesiftfeatures -x\n");
  printf("\n");
  exit(1);
}

Feature getFeatureFromKeypointN(KeypointN* p)
{
  vector<int> desc;
  for (int d = 0 ; d < p->dim ; d++) 
    {
      desc.push_back(p->descriptor[d]);
    }
  Feature feat(p->x, p->y, p->scale, p->orientation, desc);
  return feat;
}

FeatureMatchSetGroup *getFeatureMatchSetGroup(ArrayList* matchsets)
{
  FeatureMatchSetGroup *res = new FeatureMatchSetGroup();
  int count = ArrayList_Count(matchsets);
  for (int i = 0 ; i < count ; i++) 
    {
      MatchSet* ms = (MatchSet*) ArrayList_GetItem(matchsets, i);
      FeatureMatchSet set;
      set.firstscan = ms->file1;
      set.secondscan = ms->file2;
      ArrayList* matches = ms->matches;
      int mcount = ArrayList_Count(ms->matches);
      for (int j = 0 ; j < mcount ; j++) 
	{
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

/**
 *   parseArgs - reade the comand line options
 */
int parseArgs(int argc, char **argv, string &output, int &maxmatches, vector<string> &inputfiles)
{
  int c;
  opterr = 0;
  //read the command line and get the options
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
	  fprintf (stderr, "Unknown option character `%c'\n", optopt);
	return 1;
      case ':':
      default:
	usage(argc, argv);
      }
  
  if (output.empty())  
    {
      printf("You must provide an output file\n");
      usage(argc, argv);
    }
  
  if (optind > argc - 2) 
    {
      printf("%d %d\n", optind, argc);
      printf("Too few input files. At least two are required.\n");
      usage(argc, argv);
    }

  for (int index = optind; index < argc; index++) 
    {
      inputfiles.push_back(argv[index]);
    }
  
  return 1;
}


/**
 *   main - reads the xml files and match sift featurs and create the .matches files 
 */
int main (int argc, char* argv[])
{
  vector<string> inputfiles;
  string output;
  int maxmatches = 0;
  vector<string>::iterator it;
  
  parseArgs(argc, argv, output, maxmatches, inputfiles);
  cout<<endl;
  cout<<"matchsiftfeatures will procees with the following parameters: "<<endl;
  cout<<"Output: "<<output<<endl;
  cout<<"Maxmatches: "<<maxmatches<<endl;
  cout<<"Inputfiles :"<<endl;
  for (it = inputfiles.begin() ; it != inputfiles.end() ; it++) 
    {
      cout<<"    "<<*it<<endl;
    }
  cout<<endl;

  ArrayList* keyfiles = ArrayList_new0(NULL);
  for (it = inputfiles.begin() ; it != inputfiles.end() ; it++) 
    {
      string temp = *it;
      ArrayList_AddItem(keyfiles, (void*) temp.c_str());
    }
  
  MultiMatch* mm = MultiMatch_new0 ();
  
  WriteLine ("Loading keyfiles");
  MultiMatch_LoadKeysets (mm, keyfiles);
  
  WriteLine ("\nMatching...");
  clock_t start = clock();
  ArrayList* msList = MultiMatch_LocateMatchSets (mm, 3, maxmatches, false, false);
  clock_t end = clock();
  double time = ((double)((int) end - (int) start)) / CLOCKS_PER_SEC;
  
  cout << "Matched in time: " << time << endl;
  
  FeatureMatchSetGroup *msg = getFeatureMatchSetGroup(msList);
  
  msg->serialize(output.c_str());
  
  delete msg;
  ArrayList_delete(keyfiles);
  MultiMatch_delete(mm);
  
  return 0;
}
