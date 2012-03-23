/**
 * @file
 * @brief calculate  rectangle bounding box and minimal unregular bounding box for cluster
          the rectangle bounding box can be inclined therefore it can better represent cluster 
		  and vehicle direction,the result is stored as a clusterfeature in clusterFeatureArray
 *
 * @author Quanwen Zhu, Wuhan University, China
 */
#include <vector>
#include "veloslam/gridcluster.h"


using namespace std;

 struct PointID
{
	int cellIndex;
	int pointIndexInCell;
};

 struct BesiegePoint
{
	double x;
	double y;
	double angle;
	PointID pointID; 
};

 struct OrientedBoundingBox
{
	
	double angle;
	double length;
	double width;
	double circumference;
	double area;
	double newMaxXPointX;
	double newMaxXPointY;
	
	double newMaxYPointX;
	double newMaxYPointY;
	
	double newMinXPointX;
	double newMinXPointY;

	double newMinYPointX;
	double newMinYPointY;
	PointID boundingPointID[4];
};


class BoundingBox
{
public: 
		BoundingBox(void);
		~BoundingBox(void);
		void Initial(void);
		double CalDirectionTwoPoints(double XPos1,double YPos1,double XPos2,double YPos2);

		void CalCoordsAftRotation(double x,double y,double *newX,double *newY,double theta);

		void CalAllBoundingBox( cluster &gluClusterData);

		void CalBestRectangleBox(cluster &gluClusterData,clusterFeature &glu);

		void CalMinBoundingBox(cluster &gluClusterData,clusterFeature &glu);

private : 
		  bool calCandidateBox;


		   /**
		   * all rectangle bounding box candidates 
		   */
		  vector <OrientedBoundingBox> allCandBox;

		   /**
		   * unregular minBounding box points 
		   */
		  vector <BesiegePoint> minBoundingBox;

};
