/**
 *
 *	Author: Darko Makreshanski
 *  Maintained : HamidReza Feb 12, 2011
 *	this file is a modified version of GenerateKeys.c from the autopano-sift-C library
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

void usage(int argc, char** argv)
{
  printf("\n");
  printf("USAGE: %s [ -o FILE.keyset] [ -x FILE.keyxml ] -i INPUT_IMAGE\n", argv[0]);
  printf("\n");
  printf("\n");
  printf("\tOptions:\n");
  printf("\t\t-o FILE.keyset\t\tserialize the FeatureSet class\n");
  printf("\t\t-x FILE.keyxml\t\tset output file for keys in xml format\n");
  printf("\t\t-i INPUT_IMAGE\t\tinput image file -m\n");
  printf("\n");
  exit(1);
}

ImageMap* getImageMapFromPanoramaMap(PanoramaMap* map)
{
  assert(map);
  ImageMap* result = ImageMap_new(map->width, map->height);
  for (unsigned int x = 0; x < map->width; x += 1)
    {
      for (unsigned int y = 0; y < map->height; y += 1)
	{
	  //put the refelectance to the ImageMap
	  result->values[x][map->height - y - 1] = map->data[x][y].value;
	}
    }
  return result;
}

FeatureSet* getFeatureSet(ArrayList* featlist, std::string scanid)
{
  FeatureSet* set = new FeatureSet();
  set->scanid = scanid;
  for (unsigned int i = 0; i < featlist->count; i += 1)
    {
      KeypointN* p = (KeypointN* ) ArrayList_GetItem(featlist, i);
      vector<int> desc;
      for (int d = 0 ; d < p->dim ; d++) 
	{
	  desc.push_back(p->descriptor[d]);
	}
      Feature feat(p->x, p->y, p->scale, p->orientation, desc);
      set->features.push_back(feat);
    }
  return set;
}

int main (int argc, char* argv[])
{
  int downRes = 0;
  char *input = NULL;
  char *outputxml = NULL;
  char *output = NULL;
  
  WriteLine("SIFT Keypoint Generation\n"); //, version %s\n", PACKAGE_VERSION);
  
  int c;
  
  opterr = 0;
  
  while ((c = getopt (argc, argv, ":x:o:i:m:")) != -1)
    switch (c)
      {
      case 'x':
	outputxml = optarg;
	break;
      case 'o':
	output = optarg;
	break;
      case 'i':
	input = optarg;
	break;
      case 'm':
	downRes = atoi(optarg);
	if (downRes <= 0) {printf("Invalid size for MIN_DIM\n"); usage(argc, argv);}
	break;
      case '?':
	fprintf (stderr, "Option -%c requires an argument.\n", optopt);
	usage(argc, argv);
	break;
      case ':':
      default:
	fprintf (stderr, "Unknown option character `%c'.\n", optopt);
      usage(argc, argv);
      }
  
  if (input == NULL) 
    {
      printf("You must provide an input file\n");
      usage(argc, argv);
    }
  
  if (outputxml == NULL && output == NULL) 
    {
      printf("You must provide either xml (-x) or serialization output (-o)\n");
      usage(argc, argv);
    }
  
  //	 1. load the image file
  //	WriteLine ("opening %s", argv[1]);
  
  //	DisplayImage* pic = DisplayImage_new(input);
  //	int pW = pic->width;
  //	int pH = pic->height;
  //
  //	double startScale = 1.0;
  //	if (downRes > 0) {
  //			startScale = DisplayImage_ScaleWithin(pic, downRes);
  //			WriteLine ("Scaled picture, starting with scale %0.04f",
  //				    startScale);
  //	}
  //
  //	ImageMap* picMap = DisplayImage_ConvertToImageMap(pic);
  //	DisplayImage_delete(pic);
  
  //changed from converting from map to converting to image because the output was inconsistent

  PanoramaMap map(input);
  ImageMap* picMap = getImageMapFromPanoramaMap(&map);
  int pW = map.width;
  int pH = map.height;
  
  // 2. find the features
  LoweFeatureDetector* lf = LoweFeatureDetector_new0();
  //	if (argc > 3) {
  //		LoweFeatureDetector_DetectFeaturesDownscaled (lf, picMap, 0, 1.0 / startScale);
  //	} else
  
  cout << "Detecting Features...\n";
  clock_t start = clock();
  LoweFeatureDetector_DetectFeatures (lf, picMap);
  clock_t end = clock();
  double time = ((double)((int) end - (int) start)) / CLOCKS_PER_SEC;
  WriteLine ("found %d global keypoints with time: %f\n",
	     ArrayList_Count(LoweFeatureDetector_GlobalNaturalKeypoints(lf)), time);
  
  if (outputxml != NULL) 
    {
      KeypointXMLWriter_WriteComplete ((char*) map.scanid.c_str(), pW, pH, outputxml,
				       LoweFeatureDetector_GlobalNaturalKeypoints(lf));
      WriteLine("Wrote XML file to %s\n", outputxml);
    }
  
  if (output != NULL) 
    {
      FeatureSet *set = getFeatureSet(LoweFeatureDetector_GlobalNaturalKeypoints(lf), map.scanid);
      set->serialize(output);
      WriteLine("Wrote FeatureSet file to %s\n", output);
      delete set;
    }
  
  //	ImageMap_delete(picMap);
  LoweFeatureDetector_delete(lf);
  return 0;
}
