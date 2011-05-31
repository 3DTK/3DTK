
#ifndef AUTOPANOS_SIFT_H
#define AUTOPANOS_SIFT_H

#ifdef HAS_CONFIG_H
#include "config.h"
#else
#define PACKAGE_VERSION "2.5.1"
#endif

// include windows.h here
// to prevent the global namespace to become polluted with 
// badly named Windows macros


#ifdef __cplusplus
extern "C" {
#endif

#if defined(_WIN32)
# define VC_EXTRALEAN
#ifndef NOMINMAX
# define NOMINMAX
#endif
# include <windows.h>
# undef NOMINMAX
# ifdef DIFFERENCE
#  undef DIFFERENCE
# endif
#endif 

#ifdef HAS_PANO13
#include "pano13/filter.h"
#include "pano13/panorama.h"
#else
#include "pano12/filter.h"
#include "pano12/panorama.h"
#endif
#include "math.h"


#define bool int
#define true 1
#define false 0
#ifndef min
  #define min(x,y) ((x)<(y)?(x):(y))
#endif
#ifndef max
  #define max(x,y) ((x)>(y)?(x):(y))
#endif
#ifndef abs
  #define abs(x) ((x)>0?(x):(-x))
#endif

#define Double_PositiveInfinity (1e+308)
#define Double_NegativeInfinity (-1e+308)

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

void Write(const char* fmt, ...);
void WriteLine(const char* fmt, ...);
void WriteError(const char* fmt, ...);
void FatalError(const char* fmt, ...);

char* FileNameToFullPath(char* filename);
char* FullPathToFileName(char* fullpath);


int** IntMap_new(int xDim, int yDim);
void IntMap_delete(int** self);

float** FloatMap_new(int xDim, int yDim);
void FloatMap_delete(float** self);

double** DoubleMap_new(int xDim, int yDim);
void DoubleMap_delete(double** self);

void*** PtrMap_new(int xDim, int yDim);
void PtrMap_delete(void*** self, void (* deletefn)(void*));

typedef struct Random Random;
struct Random {
    int seed;
};
Random* Random_new0();
void Random_delete(Random*);
int Random_Next(Random*, int min, int max);
double Random_NextDouble(Random*);


typedef struct SimpleMatrix SimpleMatrix;
struct SimpleMatrix {
	int xDim;
	int yDim;
	double** values;
};
SimpleMatrix* SimpleMatrix_new0();
SimpleMatrix* SimpleMatrix_new(int,int);
void SimpleMatrix_init(SimpleMatrix* self, int,int);
void SimpleMatrix_delete(SimpleMatrix*);
SimpleMatrix* SimpleMatrix_clone (SimpleMatrix* self);
double SimpleMatrix_GetValue(SimpleMatrix* self, int y, int x);
void SimpleMatrix_SetValue(SimpleMatrix* self, int y, int x, double value );
SimpleMatrix* SimpleMatrix_Mul(SimpleMatrix* m1, SimpleMatrix* m2);
double SimpleMatrix_Dot (SimpleMatrix* self, SimpleMatrix* m);
void SimpleMatrix_Negate (SimpleMatrix* self);
void SimpleMatrix_Inverse (SimpleMatrix* self);
void SimpleMatrix_SolveLinear (SimpleMatrix* self, SimpleMatrix* vec);
void SimpleMatrix_SwapRow (SimpleMatrix* self, int r1, int r2);
char* SimpleMatrix_ToString (SimpleMatrix* self);


typedef struct ArrayList ArrayList;
struct ArrayList {
	int count;
	int reserved;
	void** items;
	void* deletefn;
	};
ArrayList* ArrayList_new0(void *);
void ArrayList_init(ArrayList* self, void* deletefn);
ArrayList* ArrayList_new(int count, void* deletefn);
void ArrayList_delete(ArrayList* self);
ArrayList* ArrayList_clone(ArrayList* self);
int ArrayList_Count(ArrayList* self);
void* ArrayList_GetItem(ArrayList* self, int index);
void ArrayList_SetItem(ArrayList* self, int index, void* value);
void ArrayList_AddItem(ArrayList* self, void* value);
void ArrayList_AddRange(ArrayList* self, ArrayList* list);
void ArrayList_Copy(ArrayList* self, int start, ArrayList* dest, int offset, int len);
void ArrayList_RemoveAt(ArrayList* self, int index);
void ArrayList_RemoveItem(ArrayList* self, void* value);
void ArrayList_RemoveRange(ArrayList* self, int start, int count);
bool ArrayList_Contains(ArrayList* self, void* value);
int ArrayList_IndexOf(ArrayList* self, void* value);
typedef struct IComparator IComparator;
struct IComparator {
    int (*compareTo)(IComparator* self, const void*, const void*);
};
void ArrayList_Sort(ArrayList* self, IComparator*);


typedef struct HashTable HashTable;
struct HashTable {
	ArrayList* keys;
	ArrayList* values;
};
HashTable* HashTable_new0(void* delete_key, void* delete_value);
void HashTable_delete(HashTable* self);
void HashTable_AddItem(HashTable* self, void* key, void* value);
void* HashTable_GetItem(HashTable* self, void* key);
void HashTable_SetItem(HashTable* self, void* key, void* value);
bool HashTable_Contains(HashTable* self, void* key);

typedef struct SortedLimitedList SortedLimitedList;
struct SortedLimitedList {
	ArrayList base;
	int max;
	IComparator comparator;
	void (*deletefn)(void*);
};
SortedLimitedList* SortedLimitedList_new0 ();
SortedLimitedList* SortedLimitedList_new (int, void* deletefn);
void SortedLimitedList_delete (SortedLimitedList* self);
void SortedLimitedList_SetItem (SortedLimitedList* self, int idx, void* value);
int SortedLimitedList_Count (SortedLimitedList* self);
void* SortedLimitedList_GetItem (SortedLimitedList* self, int i);
void SortedLimitedList_RemoveAt (SortedLimitedList* self, int i);
int SortedLimitedList_AddItem (SortedLimitedList* self, void* value);

typedef struct IKDTreeDomain IKDTreeDomain;

// The interface to be implemented by all data elements within the
// kd-tree. As every element is represented in domain space by a
// multidimensional vector, the interface provides readonly methods to
// access into this vector and to get its dimension.
// kept for APSCpp as KeyPointN uses it

struct IKDTreeDomain {
	int (*getDimensionCount)(IKDTreeDomain* self);
		int (*getDimensionElement)(IKDTreeDomain* self, int dim);
};


#ifdef APSCpp_useANN	// declarations for using ANN kd-tree

	typedef void KDTree;
    void KDTree_delete (KDTree* self);

#else			// declarations for using Nowozin KDTree

	int IKDTreeDomain_GetDimensionCount(IKDTreeDomain* self);
	int IKDTreeDomain_GetDimensionElement(IKDTreeDomain* self, int dim);


	typedef struct KDTreeBestEntry KDTreeBestEntry;
	typedef struct KDTreeHREntry KDTreeHREntry;
	typedef struct HyperRectangle HyperRectangle;
	typedef struct KDTree KDTree;
	struct KDTreeHREntry {
		double dist;
		HyperRectangle* rect;
		IKDTreeDomain* pivot;
		KDTree* tree;
	};

	struct HyperRectangle {
		int* leftTop;
		int* rightBottom;
		int dim;
		int ref;
	};

	HyperRectangle* HyperRectangle_new0();
	void HyperRectangle_delete(HyperRectangle* self);
	HyperRectangle* HyperRectangle_new (int dim);
	HyperRectangle* HyperRectangle_clone (HyperRectangle* self);
	HyperRectangle* HyperRectangle_ref (HyperRectangle* self);
	void HyperRectangle_unref (HyperRectangle* self);
	HyperRectangle* HyperRectangle_CreateUniverseRectangle (int dim);
	double HyperRectangle_Distance (HyperRectangle* self, IKDTreeDomain* target);
	bool HyperRectangle_IsInReach (HyperRectangle* self, IKDTreeDomain* target, double distRad);

	struct KDTreeBestEntry {
		// Distance between this neighbour point and the target point.
		double distance;
		int distanceSq;
		bool squared;
		// The neighbour.
		IKDTreeDomain* neighbour;
		
	};
	IKDTreeDomain* KDTreeBestEntry_Neighbour(KDTreeBestEntry* self);
	void KDTreeBestEntry_delete(KDTreeBestEntry* self);

	struct KDTree {
		// The current element
		IKDTreeDomain* dr;

		// The splitting dimension for subtrees.
		int splitDim;

		// The left and the right kd-subtree.
		KDTree* left;
		KDTree* right;
	};
    void KDTree_delete (KDTree* self);
	int KDTree_DistanceSq (IKDTreeDomain* t1, IKDTreeDomain* t2);
	IKDTreeDomain* KDTree_GoodCandidate (ArrayList* exset, int* splitDim);
	KDTree* KDTree_CreateKDTree (ArrayList* exset);
	KDTree* KDTree_new0 ();
	IKDTreeDomain* KDTree_NearestNeighbour (KDTree* self, IKDTreeDomain* target, double* resDist);
	IKDTreeDomain* KDTree_NearestNeighbourI (KDTree* self, IKDTreeDomain* target, HyperRectangle* hr,
						 double maxDistSq, double* resDistSq, ArrayList* hrl);
	SortedLimitedList* KDTree_NearestNeighbourList (KDTree* self, IKDTreeDomain* target,
						double* resDist, int q);
	IKDTreeDomain* KDTree_NearestNeighbourListI (KDTree* self, SortedLimitedList* best,
			int q, IKDTreeDomain* target, HyperRectangle* hr, double maxDistSq,
							 double* resDistSq, ArrayList* hrl);
	SortedLimitedList* KDTree_NearestNeighbourListBBF (KDTree* self, IKDTreeDomain* target,
						   int q, int searchSteps);
	IKDTreeDomain* KDTree_NearestNeighbourListBBFI (KDTree* self, SortedLimitedList* best,
							int q, IKDTreeDomain* target, HyperRectangle* hr, int maxDistSq,
							int* resDistSq, SortedLimitedList* searchHr, int* searchSteps, 
							ArrayList* hrl);

#endif //ndef APSCpp_useANN  // end of kd-tree declarations

typedef struct Image DisplayImage;

struct ImageMap {
	int xDim;
	int yDim;
	float** values;
};
typedef struct ImageMap ImageMap;

DisplayImage* DisplayImage_new0();
DisplayImage* DisplayImage_new(char* filename);
void DisplayImage_delete(Image* self);
double DisplayImage_ScaleWithin(DisplayImage* self, int);
ImageMap* DisplayImage_ConvertToImageMap(DisplayImage*);
DisplayImage* DisplayImage_Carve(DisplayImage* self, int x, int y, int w, int h);

ImageMap* ImageMap_new0();
ImageMap* ImageMap_new(int width, int height);
void ImageMap_delete(ImageMap* self);
void ImageMap_Save(ImageMap* self, char* filename, char* comment);
double ImageMap_ScaleWithin(ImageMap* self, int);
ImageMap* ImageMap_ScaleDouble(ImageMap* self);
ImageMap* ImageMap_ScaleHalf(ImageMap* self);
ImageMap* ImageMap_GaussianConvolution(ImageMap* self, double);
ImageMap* ImageMap_Add(ImageMap* f1, ImageMap* f2);
ImageMap* ImageMap_Sub(ImageMap* f1, ImageMap* f2);
ImageMap* ImageMap_Mul(ImageMap* f1, ImageMap* f2);
void ImageMap_Normalize(ImageMap* self);
ImageMap* ImageMap_GaussianConvolution(ImageMap* self, double sigma);

static inline double ImageMap_GetPixel(ImageMap* self, int x, int y)
{
	return self->values[x][y];
}

static inline void ImageMap_SetPixel(ImageMap* self, int x, int y, float value)
{
    self->values[x][y]=value;
}




typedef struct ConvLinearMask ConvLinearMask;
struct ConvLinearMask {
	int Dim;
	int Middle;
	double* items;
	double MaskSum;
};

typedef struct GaussianConvolution GaussianConvolution;
struct GaussianConvolution {
	ConvLinearMask* mask;
};

GaussianConvolution* GaussianConvolution_new0 ();
void GaussianConvolution_delete (GaussianConvolution* );
GaussianConvolution* GaussianConvolution_new1 (double sigma);
GaussianConvolution* GaussianConvolution_new2 (double sigma, int dim);
ImageMap* GaussianConvolution_Convolve (GaussianConvolution*, ImageMap*);

ImageMap* ConvolutionFilter_Convolve (ImageMap* img, ConvLinearMask* mask);
void ConvolutionFilter_Convolve1D (ImageMap* dest, ConvLinearMask* mask,
				   ImageMap* src, int dir);
double ConvolutionFilter_CalculateConvolutionValue1D (ImageMap* src,
						      ConvLinearMask* mask, int n, int p, int maxN, int maxP, int dir);


typedef struct IRANSACModel IRANSACModel;
struct IRANSACModel {
	IRANSACModel* (*clone)(IRANSACModel*);
	void (*deletefn)(IRANSACModel*);

	// Fit the model to the samples given. The number of samples is equal
	// to or larger than the smallest number of points required for a fit
	// ('n').
	// Return true if the fit can be done, false otherwise.
	bool (*fitModel)(IRANSACModel*, ArrayList*);

	// Return the fitting error of a single point against the current
	// model.
	double (*fittingErrorSingle)(IRANSACModel*, void*);

	// Threshhold the given fit error of a point.
	// Return true if the fitting error is small enough and the point is
	//     fitting.
	// Return false if the point is not fitting.
	bool (*threshholdPoint)(IRANSACModel*, double fitError);

	// The overall fitting error of all points in FittingGround. This
	// value is calculated by averaging all individual fitting errors of
	// the points in the FittingGround.
	double (*getFittingErrorSum)(IRANSACModel*);
	void (*setFittingErrorSum)(IRANSACModel*, double);

	// All the points used to fit. Has to be set explicitly.
	ArrayList* (*getFittingGround)(IRANSACModel*);
	void (*setFittingGround)(IRANSACModel*, ArrayList*);
	int (*compareTo)(IComparator*, IRANSACModel*, IRANSACModel*);
};

bool IRANSACModel_FitModel(IRANSACModel*, ArrayList*);
IRANSACModel* IRANSACModel_clone(IRANSACModel*);
void IRANSACModel_delete(IRANSACModel* self);
double IRANSACModel_FittingErrorSingle(IRANSACModel*, void*);
bool IRANSACModel_ThreshholdPoint (IRANSACModel*, double);
void IRANSACModel_SetFittingErrorSum(IRANSACModel*, double);
double IRANSACModel_GetFittingErrorSum(IRANSACModel*);
void IRANSACModel_SetFittingGround(IRANSACModel*, ArrayList*);
void IRANSACModel_CompareTo(IComparator*, void*, void*);


typedef struct RANSAC RANSAC;
struct RANSAC {
    	// Smallest number of points to be able to fit the model.
	int n;

	// The number of iterations required.
	int k;
};
RANSAC* RANSAC_new0();
RANSAC* RANSAC_new(int n, int k);
void RANSAC_delete(RANSAC* );
ArrayList* RANSAC_FindModels(RANSAC*, IRANSACModel*, ArrayList*, int);
ArrayList* RANSAC_Sort(RANSAC*);
int RANSAC_GetKFromGoodfraction (int n, double goodFraction, int sdM);

typedef struct OctavePyramid OctavePyramid;
typedef struct DScaleSpace DScaleSpace;

struct OctavePyramid {
	bool Verbose;
	// Holds DScaleSpace objects, ordered by descending image size.
	ArrayList* octaves;
};
OctavePyramid* OctavePyramid_new0();
void OctavePyramid_delete(OctavePyramid* self);
int OctavePyramid_Count(OctavePyramid* self);
int OctavePyramid_BuildOctaves(OctavePyramid* self, ImageMap*, double, int, double, int);
DScaleSpace* OctavePyramid_GetScaleSpace(OctavePyramid* self, int);


struct PointLocalInformation {
	// Sub-pixel offset relative from this point. In the range of [-0.5 ; 0.5]
	double fineX, fineY;
	// Relative scale adjustment to the base image scale
	double scaleAdjust;
	double dValue;
};
typedef struct PointLocalInformation PointLocalInformation;
PointLocalInformation* PointLocalInformation_new0 ();
void PointLocalInformation_delete (PointLocalInformation* self);
PointLocalInformation* PointLocalInformation_new3 (double fineS, double fineX, double fineY);


// A single point in scale space, used in keypoint creation to describe an
// exact position in scalespace and additional information about that point.
// Should not be used outside.
typedef struct ScalePoint ScalePoint;
struct ScalePoint {
	int x;
	int y;
	int level;
	// Sub-pixel level information from the Localization step are put here
	PointLocalInformation* local;
};
ScalePoint* ScalePoint_new0 ();
void ScalePoint_delete (ScalePoint* self);
ScalePoint* ScalePoint_new3 (int x, int y, int level);


// A single keypoint, the final result of keypoint creation. Contains the
// keypoint descriptor and position.
typedef struct Keypoint Keypoint;
struct Keypoint {
	ImageMap* image;
	double y, x;
	double imgScale;	// The scale of the image the keypoint was found in
	// The absolute keypoint scale, where 1.0 is the original input image
	double scale;
	double orientation;
	// The actual keypoint descriptor.
	bool hasFV;
	double* featureVector;
	int featureVectorLength;
	int xDim, yDim, oDim;
};

Keypoint*  Keypoint_new0();
void Keypoint_delete(Keypoint* self);
Keypoint*  Keypoint_new (ImageMap* image, double x, double y, double imgScale,
			 double kpScale, double orientation);
double Keypoint_FVGet (Keypoint* self, int xI, int yI, int oI);
void Keypoint_FVSet (Keypoint* self, int xI, int yI, int oI, double value);
int Keypoint_FVLinearDim(Keypoint* self);
double Keypoint_FVLinearGet (Keypoint* self, int idx);
void Keypoint_FVLinearSet (Keypoint* self, int idx, double value);
void Keypoint_CreateLinearVector (Keypoint* self, int dim);
void Keypoint_CreateVector (Keypoint* self, int xDim, int yDim, int oDim);


// A single normalized and natural number keypoint. Contains the descriptor,
// position and orientation.
typedef struct KeypointN KeypointN;
struct KeypointN {
	IKDTreeDomain domain;
	double x, y;
	double scale;
	double orientation;

	int dim;
	int* descriptor;
};
KeypointN* KeypointN_new0();
void KeypointN_delete(KeypointN* self);
KeypointN* KeypointN_new(Keypoint* kp);
KeypointN* KeypointN_clone(KeypointN* self);
void KeypointN_CreateDescriptor(KeypointN* self);
int KeypointN_GetDimensionCount(KeypointN* self);
int KeypointN_GetDimensionElement(KeypointN* self, int n);


struct DScaleSpace {
	bool Verbose;
	DScaleSpace* Down;
	DScaleSpace* Up;

	// The original gaussian blurred source image this level was started with.
	// Needed for keypoint generation.

	ImageMap* baseImg;
	double basePixScale;

	// The smoothed gaussian images, all the base (lower scale) relative to
	// the DoG spaces below.
	ArrayList* imgScaled;

	ArrayList* magnitudes;
	ArrayList* directions;

	// The DoG spaces.
	ArrayList* spaces;
};
DScaleSpace* DScaleSpace_new0();
void DScaleSpace_delete(DScaleSpace* self);
ImageMap* DScaleSpace_GetGaussianMap (DScaleSpace* self, int idx);
ImageMap* DScaleSpace_LastGaussianMap(DScaleSpace* self);
int DScaleSpace_Count(DScaleSpace* self);
ImageMap* DScaleSpace_Map(DScaleSpace* self, int idx);
ArrayList* DScaleSpace_GenerateKeypoints(DScaleSpace* self, ArrayList*, int, double);
ArrayList* DScaleSpace_GenerateKeypointSingle (DScaleSpace* self,
					       double imgScale, ScalePoint* point,
					       int binCount, double peakRelThresh, int scaleCount,
					       double octaveSigma);
bool DScaleSpace_InterpolateOrientation (DScaleSpace* self,
					 double left, double middle,
					 double right, double* degreeCorrection, double* peakValue);
int DScaleSpace_FindClosestRotationBin (DScaleSpace* self, int binCount, double angle);
void DScaleSpace_AverageWeakBins (DScaleSpace* self, double* bins, int binCount);
ArrayList* DScaleSpace_CreateDescriptors (DScaleSpace* self,
					  ArrayList* keypoints,
					  ImageMap* magnitude, ImageMap* direction,
					  double considerScaleFactor, int descDim, int directionCount,
					  double fvGradHicap);
void DScaleSpace_CapAndNormalizeFV (DScaleSpace* self, Keypoint* kp, double fvGradHicap);
bool DScaleSpace_IsInCircle (int rX, int rY, int radiusSq);
ArrayList* DScaleSpace_FilterAndLocalizePeaks (DScaleSpace* self, ArrayList* peaks, double edgeRatio,
					       double dValueLoThresh, double scaleAdjustThresh, int relocationMaximum);
bool DScaleSpace_LocalizeIsWeak (DScaleSpace* self, ScalePoint* point, int steps, int** processed);
bool DScaleSpace_IsTooEdgelike (DScaleSpace* self, ImageMap* space, int x, int y, double r);
SimpleMatrix* DScaleSpace_GetAdjustment (DScaleSpace* self, ScalePoint* point,
					 int level, int x, int y, double* dp);
ArrayList* DScaleSpace_FindPeaks(DScaleSpace* self, double);
ArrayList* DScaleSpace_FindPeaksThreeLevel (DScaleSpace* self, ImageMap* below, ImageMap* current,
					    ImageMap* above, int curLev, double dogThresh);
void DScaleSpace_GenerateMagnitudeAndDirectionMaps(DScaleSpace* self);
void DScaleSpace_ClearMagnitudeAndDirectionMaps(DScaleSpace* self);
void DScaleSpace_BuildDiffMaps (DScaleSpace* self);
void DScaleSpace_BuildGaussianMaps (DScaleSpace* self, ImageMap* first, double firstScale,
				    int scales, double sigma);
void DScaleSpace_CheckMinMax (DScaleSpace* self, ImageMap* layer, double c, int x, int y,
			      bool* IsMin, bool* IsMax, bool cLayer);


typedef struct LoweFeatureDetector LoweFeatureDetector;
struct LoweFeatureDetector {
    ArrayList* globalKeypoints;
    ArrayList* globalNaturalKeypoints;
    OctavePyramid* pyr;
};

void LoweFeatureDetector_SetPrintWarning(bool value);
void LoweFeatureDetector_SetVerbose(bool value);
double LoweFeatureDetector_SetPreprocSigma( double newsigma );

LoweFeatureDetector* LoweFeatureDetector_new0();
void LoweFeatureDetector_delete(LoweFeatureDetector* );
ArrayList* LoweFeatureDetector_GlobalKeypoints(LoweFeatureDetector* );
ArrayList* LoweFeatureDetector_GlobalNaturalKeypoints(LoweFeatureDetector* );
int LoweFeatureDetector_DetectFeaturesDownscaled(LoweFeatureDetector* self, ImageMap*, int, double);
int LoweFeatureDetector_DetectFeatures(LoweFeatureDetector* self, ImageMap*);


typedef struct KeypointXMLList KeypointXMLList;
struct KeypointXMLList {
    char* imageFile;
    ArrayList* array;
    int xDim;
    int yDim;
};
KeypointXMLList* KeypointXMLList_new0();
void KeypointXMLList_delete(KeypointXMLList* self);
KeypointXMLList* KeypointXMLList_new (char* imageFile, int xDim, int yDim, ArrayList* list);
KeypointXMLList* KeypointXMLList_new2 (ArrayList* list, int xDim, int yDim);
void KeypointXMLList_Add(KeypointXMLList* self, KeypointN* kp);
KeypointXMLList* KeypointXMLReader_ReadComplete(char*);
KeypointXMLList* KeypointXMLReader_ReadComplete2(char*, bool);
void KeypointXMLWriter_WriteComplete(char*, int, int, char*, ArrayList*);
void KeypointXMLWriter_WriteComplete2(char*, int, int, char*, ArrayList*, bool);


typedef struct Match Match;
struct Match {
	KeypointN* kp1;
	KeypointN* kp2;
	double dist1;
	double dist2;
};
Match* Match_new0();
Match* Match_new(KeypointN* kp1, KeypointN* kp2, double dist1, double dist2);
Match* Match_clone (Match* self);
void Match_delete(Match* self);

typedef struct MatchWeighter MatchWeighter;
struct MatchWeighter {
	IComparator comparator;
	double distExp;
	double quotExp;
};
MatchWeighter* MatchWeighter_new0();
MatchWeighter* MatchWeighter_new(double distExp, double quotExp);
void MatchWeighter_delete(MatchWeighter* self);
int MatchWeighter_CompareTo (MatchWeighter* self, Match* m1, Match* m2);
double MatchWeighter_OverallFitness(MatchWeighter* self, Match* m);

typedef struct Position Position;
struct Position {
	double yaw;
	double pitch;
	double rotation;
};
Position* Position_new0();
void Position_delete(Position* self);
Position* Position_new(double yaw, double pitch, double rotation);
char* Position_ToString (Position* self);

typedef struct AffineTransform2D AffineTransform2D;
struct AffineTransform2D {
	struct SimpleMatrix base;
	// The relative angle of both images in the affine transformation.
	double rotationAngle;
	// The angle between the horizon and the line through the centers of both
	// images.
	double centerAngle;
	double shiftWidth;
};
AffineTransform2D* AffineTransform2D_new0();
void AffineTransform2D_delete(AffineTransform2D* self);
AffineTransform2D* AffineTransform2D_clone(AffineTransform2D* self);
int AffineTransform2D_GetShiftWidth(AffineTransform2D* self);
AffineTransform2D*  AffineTransform2D_BuildTransformFromTwoPairs(double,double,double,double,double,double,double,double,int,int);

typedef struct ImageMatchModel ImageMatchModel;
struct ImageMatchModel {
	struct IRANSACModel base;

	// The two original matches we build the model on.
	Match* m1;
	Match* m2;

	double fitThresh;

	// The distance-gratifying factor in the distance relaxing formula.
	double distanceFactor;
	// The image resolution to calculate the maximum possible distance.
	int width, height;

	double fittingErrorSum;
	ArrayList* fittingGround;
	AffineTransform2D* trans;
};

ImageMatchModel* ImageMatchModel_clone(ImageMatchModel* self);
void ImageMatchModel_delete(ImageMatchModel* self);
ImageMatchModel* ImageMatchModel_new (double fitThresh, double distanceFactor,
				      int width, int height);
char* ImageMatchModel_ToString (ImageMatchModel* self);
ImageMatchModel* MatchDriver_FilterMatchSet (ArrayList* matches,
					     double distanceFactor, int width, int height);

typedef struct FilterPoint FilterPoint;
struct FilterPoint
{
	double x, y;
	void* user;
};

FilterPoint* FilterPoint_new0();
void FilterPoint_delete(FilterPoint* self);

typedef struct AreaFilter AreaFilter;

struct AreaFilter {
  int dummy; // dummy entry required by MSVC 2008	
};

AreaFilter* AreaFilter_new0();
void AreaFilter_delete(AreaFilter* self);
double AreaFilter_PolygonArea(AreaFilter* self, ArrayList* orderedPoints);
ArrayList* AreaFilter_CreateConvexHull (AreaFilter* self, ArrayList* points);
int AreaFilter_CreateHull (AreaFilter* self, ArrayList* points);
int AreaFilter_CompareLow (const FilterPoint* p1, const FilterPoint* p2);
int AreaFilter_CompareHigh (const FilterPoint* p1, const FilterPoint* p2);
bool AreaFilter_ccw (ArrayList* points, int i, int j, int k);
int AreaFilter_MakeChain (AreaFilter* self, ArrayList* points, int (*comp)(const FilterPoint*, const FilterPoint*));

typedef struct MultiMatch MultiMatch;
struct MultiMatch {
	ArrayList* keySets; // KeypointXMLList[]
	// Global k-d tree, containing all keys.
	KDTree* globalKeyKD;
	// Global key list, containing Keypoint elements
	ArrayList* globalKeys;

	// Global match list containing Match objects
	ArrayList* globalMatches;

	// Partitioned matches
	void*** matchSets;
	int imageCount;

	ArrayList* filteredMatchSets;

	bool verbose;
};

MultiMatch* MultiMatch_new0();
void MultiMatch_delete(MultiMatch* self);
void MultiMatch_LoadKeysetsFromMemory (MultiMatch* self, ArrayList* memlist);
void MultiMatch_LoadKeysets(MultiMatch* self, ArrayList* filenames);



// a set of files from the same panorama
typedef struct Component Component;
struct Component {
	ArrayList* files;
};
Component* Component_new0();
void Component_delete(Component* self);
void Component_AddComponent(Component* self, Component* comp);
bool Component_IsIncluded (Component* self, char* filename);
void Component_AddFile (Component* self, char* filename);
char* Component_ToString (Component* self);

// Matches between two images
typedef struct MatchSet MatchSet;
struct MatchSet {
	MultiMatch* parent; // inner class
	char* file1;
	char* file2;
	int xDim1, yDim1, xDim2, yDim2;
	
	ArrayList* matches;

	// The best result of the RANSAC matching
	ImageMatchModel* bestMatchFit;
	KeypointXMLList* keys1;
	KeypointXMLList* keys2;
};
MatchSet* MatchSet_new0(MultiMatch* parent);
MatchSet* MatchSet_new(MultiMatch* parent, 
		       char* file1, int xdim1, int ydim1,
		       char* file2, int xdim2, int ydim2,
		       KeypointXMLList* kp1, KeypointXMLList* kp2);
void MatchSet_delete(MatchSet* self);

ArrayList* MultiMatch_TwoPatchMatch (MultiMatch* self, ArrayList* kp1, int kp1maxX, int kp1maxY,
				   ArrayList* kp2, int kp2maxX, int kp2maxY, bool useRANSAC);
ArrayList* MultiMatch_LocateMatchSets (MultiMatch* self, 
				      int minimumMatches, int bestMatches,
				     bool useRANSAC, bool useAreaFiltration);
ArrayList* MultiMatch_CreatePointList (MultiMatch* self, ArrayList* matches);
ArrayList* MultiMatch_FilterMatchesByArea (MultiMatch* self, ArrayList* matches, int bestMatches,
					 double* areaPixels);
ArrayList* MultiMatch_FilterMatches (MultiMatch* self, ArrayList* matches, int bestMatches);
void MultiMatch_BuildGlobalKD (MultiMatch* self);
void MultiMatch_PartitionMatches (MultiMatch* self);
int MultiMatch_FindOrigin (MultiMatch* self, KeypointN* kp);
void MultiMatch_BuildGlobalMatchList (MultiMatch* self);
ArrayList* MultiMatch_ComponentCheck (MultiMatch* self, ArrayList* matches);
Component* MultiMatch_FindComponent (ArrayList* components, char* filename);

ArrayList* MatchSet_GetOriginalKeys (MatchSet* self, int which);


typedef struct BondBall BondBall;
struct BondBall {
	int dir;
	double center;
	double rotation;

	// 0 for left, 1 for right
	int bottomDefault;
	// Orientation tolerance in degrees, to still be considered of as lying in
	// the same direction.
	double bondOrientationTolerance;
	ArrayList* sets;
	MatchSet* first;
	MatchSet* last;
	ArrayList* firstRow;

	// From image filename as key to a Position class.
	HashTable* positions;
	// One image yaw move step.
	double yawStep;
};
BondBall* BondBall_new0();
void BondBall_delete(BondBall* self);
BondBall* BondBall_new(int bottomDir);
bool BondBall_IsWithinAngleDegree (double left, double right, double test);
bool BondBall_InitiateBond (BondBall* self, MatchSet* first);
bool BondBall_AddRowImage (BondBall* self, MatchSet* next);
Position* BondBall_EstimateImage (BondBall* self, MatchSet* ms);
void BondBall_StretchImages (BondBall* self, bool is360);
int BondBall_GuessBottomOrientation (ArrayList* keys, int xDim);
char* BondBall_ToString (BondBall* self);
BondBall* MultiMatch_BuildBondBall (MultiMatch* self, ArrayList* ransacFiltered, int bottomDefault);


#ifdef __cplusplus
}
#endif
#endif // AUTOPANOS_SIFT_H
