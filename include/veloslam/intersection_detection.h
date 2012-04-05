
#include "veloslam/gridcell.h"
#include "veloslam/gridcluster.h"
#include "veloslam/svm.h"

float absf(float a);

struct IntersectionFeature
{
	float angle;
	float slashLength;
	int slashEndColumn;
	int slashEndRow;
};

class IntersectionDetection
{
public:
	IntersectionDetection();
	~IntersectionDetection();


	int GetPointData();
	int CalcRadAndTheta();
	int CalPointCellPos(double x,double y,double z ,int * column,int * row);
	int CalCellMinDis();
	int TransferToCellArray();

	int CalcAllCellFeature();
	int CalcCellFeature(cell& cellobj, cellFeature& f);
	int CalWideSlashEdge_For_RoadShape(float Angle,int startColumID,int startRowID,float maxLength,float wide);
	int DetectIntersection();


private:

  int cellSize;
  int columnNum;
  int cellNum;
  int MinRad;
  int MaxRad;


  vector <Point> allPoints_AfterRegstn;
  cellArray cellArray_AfterRegstn;
  cellFeatureArray cellFeatureArray_AfterRegstn;
  float slashWide;
  float slashMaxLength;
  float slashAngleDefinition;
  vector <float> minCellDisList;
  vector <IntersectionFeature> intersectionFeature;


 /* clusterArray clusterArrayAfterRegstn;
  clusterFeatureArray clusterFeatureArray_AfterRegstn;
*/


};