
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

#include "AutoPanoSift.h"

int main (int argc, char* argv[])
{    
	WriteLine("SIFT Keypoint Generation, version %s\n", PACKAGE_VERSION);

	if ((argc-1) < 2 || (argc-1) > 3) {
		WriteLine ("usage: generatekeys.exe image output.key [minDim]\n");

		WriteLine ("image: Image file (any common format: JPEG, PNG, TIFF, ..)");
		WriteLine ("output.key: Output keypoint file");
		WriteLine ("    The output file can be stored in gzip compressed format by appending\n    \".gz\" to the filename.");
		WriteLine ("minDim: (optional) Downscale resolution");
		WriteLine ("    The image is repeatedly halfed in size until both width and height\n    are below 'minDim'.");
		WriteLine ("");
	
		return -1;
	}

	// 1. load the image file
	//WriteLine ("opening %s", argv[1]);

	DisplayImage* pic = DisplayImage_new(argv[1]);
	int pW = pic->width;
	int pH = pic->height;
    
	double startScale = 1.0;
	if ((argc-1) >= 3) {
		int downRes;
		if (!sscanf(argv[3], "%d", &downRes)) {
			WriteLine ("Downscale resolution \"%s\" is not valid.\nUse a positive integer.", argv[3]);
			return -1;
		}
	
		if (downRes > 0) {
			startScale = DisplayImage_ScaleWithin(pic, downRes);
			WriteLine ("Scaled picture, starting with scale %0.04f",
				    startScale);
		}
	}
    
	ImageMap* picMap = DisplayImage_ConvertToImageMap(pic);
	DisplayImage_delete(pic);

	// 2. find the features
	LoweFeatureDetector* lf = LoweFeatureDetector_new0();
	if (argc > 3) {
		LoweFeatureDetector_DetectFeaturesDownscaled (lf, picMap, 0, 1.0 / startScale);
	} else
		LoweFeatureDetector_DetectFeatures (lf, picMap);

	WriteLine ("found %d global keypoints",
		    ArrayList_Count(LoweFeatureDetector_GlobalNaturalKeypoints(lf)));
    
	KeypointXMLWriter_WriteComplete (argv[1], pW, pH, argv[2],
					 LoweFeatureDetector_GlobalNaturalKeypoints(lf));

	LoweFeatureDetector_delete(lf);
	return 0;
}


