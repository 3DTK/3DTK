/*
 * clusterboundingbox implementation
 *
 * Copyright (C) Li Wei, Li Ming
 *
 * Released under the GPL version 3.
 *
 */

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <algorithm>
#include "veloslam/debugview.h"
#include "veloslam/clusterboundingbox.h"

 bool operator<(const BesiegePoint & s1,const BesiegePoint & s2)
{
    return s1.angle < s2.angle;
}

BoundingBox::BoundingBox(void)
{
	calCandidateBox=false;
}

BoundingBox::~BoundingBox(void)
{
}

 void BoundingBox::Initial()
 {
	 calCandidateBox=false;
	 allCandBox.clear();
	 minBoundingBox.clear();
 }

void BoundingBox::CalCoordsAftRotation(double x,double y,double *newX,double *newY,double theta)
{

	 *newY=y*cosf(theta*M_PI/180)-x*sinf(theta*M_PI/180);
	 *newX=x*cosf(theta*M_PI/180)+y*sinf(theta*M_PI/180);

}


double BoundingBox::CalDirectionTwoPoints(double XPos1,double YPos1,double XPos2,double YPos2)
{
	double delt_x=XPos2-XPos1;
	double delt_y=YPos2-YPos1;
	double angle;
	double length=sqrt(delt_x*delt_x+delt_y*delt_y*1.0);
	if(delt_x<0)
		angle=360-acos(delt_y/length)*180/M_PI;
	else
		angle=acos(delt_y/length)*180/M_PI;
	return angle;
}

 void BoundingBox:: CalAllBoundingBox(cluster &gluClusterData)
 {

	 int angleInterval=2;
	 int minCircumferenceIndex;
	 double minCircumference=10000000000;;
	  OrientedBoundingBox tempBoundingBox;

	  Initial();
	  for(int i=0;i<90;i+=angleInterval)
	  {
		  tempBoundingBox.newMaxXPointX=-1000000;
		  tempBoundingBox.newMaxYPointY=-1000000;
		  tempBoundingBox.newMinXPointX=1000000;
		  tempBoundingBox.newMinYPointY=1000000;
		  for (int j = 0;  j <gluClusterData.size();j++)
		  {

			  cell* pcellObj= gluClusterData[j]->pCell;
				  for(int k=0;k<pcellObj->size();k++)
			  {

				   double newX,newY;
				  CalCoordsAftRotation((*pcellObj)[k]->x,(*pcellObj)[k]->y,&newX,&newY,i);
				  if(tempBoundingBox.newMaxXPointX<newX)
				  {
					  tempBoundingBox.newMaxXPointX=newX;
					  tempBoundingBox.newMaxXPointY=newY;
					  tempBoundingBox.boundingPointID[0].cellIndex=j;
					  tempBoundingBox.boundingPointID[0].pointIndexInCell=k;
				  }
				  if(tempBoundingBox.newMinXPointX>newX)
				  {
					  tempBoundingBox.newMinXPointX=newX;
					  tempBoundingBox.newMinXPointY=newY;
					   tempBoundingBox.boundingPointID[1].cellIndex=j;
					  tempBoundingBox.boundingPointID[1].pointIndexInCell=k;
				  }
				  if(tempBoundingBox.newMaxYPointY<newY)
				  {
					  tempBoundingBox.newMaxYPointY=newY;
					  tempBoundingBox.newMaxYPointX=newX;
					   tempBoundingBox.boundingPointID[2].cellIndex=j;
					  tempBoundingBox.boundingPointID[2].pointIndexInCell=k;
				  }
				  if(tempBoundingBox.newMinYPointY>newY)
				  {
					  tempBoundingBox.newMinYPointY=newY;
					  tempBoundingBox.newMinYPointX=newX;
					   tempBoundingBox.boundingPointID[3].cellIndex=j;
					  tempBoundingBox.boundingPointID[3].pointIndexInCell=k;
				  }
			  }
		  }
		  tempBoundingBox.length=tempBoundingBox.newMaxXPointX-tempBoundingBox.newMinXPointX;
		  tempBoundingBox.width=tempBoundingBox.newMaxYPointY-tempBoundingBox.newMinYPointY;
		  tempBoundingBox.area=(tempBoundingBox.newMaxXPointX-tempBoundingBox.newMinXPointX)*(tempBoundingBox.newMaxYPointY-tempBoundingBox.newMinYPointY);
		  tempBoundingBox.circumference=((tempBoundingBox.newMaxXPointX-tempBoundingBox.newMinXPointX)+(tempBoundingBox.newMaxYPointY-tempBoundingBox.newMinYPointY))*2;
		  tempBoundingBox.angle=i;

		/**如果旋转后得出的结果在Y轴上比X轴长，统一为X轴为长轴**/
		 if(tempBoundingBox.newMaxXPointX-tempBoundingBox.newMinXPointX<
			  tempBoundingBox.newMaxYPointY- tempBoundingBox.newMinYPointY)
		  {
			   OrientedBoundingBox temp=tempBoundingBox;
			   tempBoundingBox.angle=temp.angle+90;

			  tempBoundingBox.newMaxXPointX=temp.newMaxYPointY;
			  tempBoundingBox.newMaxXPointY=-temp.newMaxYPointX;
			  tempBoundingBox.newMaxYPointX=temp.newMinXPointY;
			  tempBoundingBox.newMaxYPointY=-temp.newMinXPointX;

			  tempBoundingBox.newMinXPointX=temp.newMinYPointY;
			  tempBoundingBox.newMinXPointY=-temp.newMinYPointX;
			  tempBoundingBox.newMinYPointX=temp.newMaxXPointY;
			  tempBoundingBox.newMinYPointY=-temp.newMaxXPointX;
		  }

		  	  allCandBox.push_back(tempBoundingBox);
	  }
 }

 void  BoundingBox::CalBestRectangleBox(cluster &gluClusterData,clusterFeature &glu)
 {
	 int minCircumferenceIndex=0;
	 int bestBoxIndex;
	 if(!calCandidateBox)
		 CalAllBoundingBox(gluClusterData);
	double minCircumference=allCandBox[0].circumference;

      for(int i=0;i<allCandBox.size();i++)
	  {
 		if(allCandBox[i].circumference<minCircumference)
		  {
		  minCircumference=allCandBox[i].circumference;
		  minCircumferenceIndex=i;
		  }
	  }
	bestBoxIndex=minCircumferenceIndex;
 	double clusterVex[4][2]={allCandBox[bestBoxIndex].newMaxXPointX,allCandBox[bestBoxIndex].newMaxYPointY,
							allCandBox[bestBoxIndex].newMinXPointX,allCandBox[bestBoxIndex].newMaxYPointY,
							allCandBox[bestBoxIndex].newMinXPointX,allCandBox[bestBoxIndex].newMinYPointY,
							allCandBox[bestBoxIndex].newMaxXPointX,allCandBox[bestBoxIndex].newMinYPointY,
	 };
	 for(int i=0;i<4;i++)
	 {
		 CalCoordsAftRotation(clusterVex[i][0],clusterVex[i][1],
			 &glu.boxVex[i][0],&glu.boxVex[i][1],360-allCandBox[bestBoxIndex].angle);
	 }
	 glu.length=allCandBox[bestBoxIndex].length;
	 glu.width=allCandBox[bestBoxIndex].width;
	 glu.boxDirection=allCandBox[bestBoxIndex].angle;

	 Draw_Inclined_Cube_GL_RGB(glu.boxVex,glu.min_z,glu.max_z,0.0,0.0,1.0,3);

 }
  void  BoundingBox::CalMinBoundingBox(cluster &gluClusterData,clusterFeature &glu)
  {
	   if(!calCandidateBox)
		 CalAllBoundingBox(gluClusterData);

	 double  minBisegeAverageX=0;
	 double minBisegeAverageY=0;
	 for(int i=0;i<allCandBox.size();i++)
	 {
		 for(int j=0;j<4;j++)
		 {
			int cellIndex=allCandBox[i].boundingPointID[j].cellIndex;
			int pointIndexInCell=allCandBox[i].boundingPointID[j].pointIndexInCell;
			 cell* pcellObj= gluClusterData[cellIndex]->pCell;
			 BesiegePoint tempPoint;
			 tempPoint.x=(*pcellObj)[pointIndexInCell]->x;
			 tempPoint.y=(*pcellObj)[pointIndexInCell]->y;
			 minBisegeAverageX+=tempPoint.x;
			 minBisegeAverageY+=tempPoint.y;
			 minBoundingBox.push_back(tempPoint);
		 }
	 }
	 minBisegeAverageX/=minBoundingBox.size();
	 minBisegeAverageY/=minBoundingBox.size();
	 for(int i=0;i<minBoundingBox.size();i++)
	 {
		 minBoundingBox[i].angle=CalDirectionTwoPoints(minBisegeAverageX,minBisegeAverageY,minBoundingBox[i].x,minBoundingBox[i].y);
	 }
	  sort(minBoundingBox.begin(),minBoundingBox.end());

 	  for(int i=0;i<minBoundingBox.size()-1;i++)
 		Draw_Line_GL_RGB(minBoundingBox[i].x,minBoundingBox[i].y,-225,minBoundingBox[i+1].x,
				 minBoundingBox[i+1].y, -225,1,0,0,3);
	    Draw_Line_GL_RGB(minBoundingBox[0].x,minBoundingBox[0].y,-225,
			 minBoundingBox[minBoundingBox.size()-1].x,minBoundingBox[minBoundingBox.size()-1].y,-225,1,0,0,3);

  }
