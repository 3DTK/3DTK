/*
 * intersection_detection implementation
 *
 * Copyright (C) Chen Long, Li Wei, Li Ming, Andreas Nuechter,
 *
 * Released under the GPL version 3.
 *
 */

#include "veloslam/intersection_detection.h"
#include "veloslam/veloscan.h"
#include <iostream>
#include <fstream>
#define  DefaultColumnSize 360

svm_model *m = svm_load_model("SegIter.model");
svm_node *nod=new svm_node[361];

IntersectionDetection::IntersectionDetection()
{
	slashWide=200;
	slashMaxLength=3500;
	slashAngleDefinition=1;

	columnNum= 360;
	cellSize= 50;
	MaxRad=6000;
	if(MaxRad%cellSize!=0)
		MaxRad=(MaxRad/cellSize+1)*cellSize;
	cellNum=MaxRad/cellSize;
	MinRad=0;

//	CalcRadAndTheta();
}

IntersectionDetection::~IntersectionDetection()
{


}

int IntersectionDetection::GetPointData()
{
	for(unsigned int i = 0; i < Scan::allScans.size(); i++)
		for (int j = 0; j < Scan::allScans[i]->get_points_red_size(); j++)
		{
			Point p;
			p.x=Scan::allScans[i]->get_points_red()[j][0];
			p.y=Scan::allScans[i]->get_points_red()[j][1];
			p.z=Scan::allScans[i]->get_points_red()[j][2];
			allPoints_AfterRegstn.push_back(p);
		}

		return 0;
}

int IntersectionDetection::CalcRadAndTheta()
{

	GetPointData();
	int i,j;
	int size=  allPoints_AfterRegstn.size();

	for(i=0; i< size; ++i)
	{
		allPoints_AfterRegstn[i].rad  = sqrt(  allPoints_AfterRegstn[i].x*allPoints_AfterRegstn[i].x
			+   allPoints_AfterRegstn[i].z*allPoints_AfterRegstn[i].z );
		allPoints_AfterRegstn[i].tan_theta  = allPoints_AfterRegstn[i].z/allPoints_AfterRegstn[i].x;
	}
	return 0;

}


int IntersectionDetection::CalPointCellPos(double x,double y,double z ,int * column,int * row)
{
	int i,j,count=0;
	float flag;
	int offset;
	int sectionSize=columnNum/8;
	float inc=(M_PI*2)/columnNum;

	vector<float> tanv;
	vector<float>::iterator result;

	for(i=0; i<sectionSize; ++i)
	{
		tanv.push_back(tan(inc*i));
	}
	double rad=sqrt(x*x+z*z);
	double tan_theta=z/x;

	int diff;
	if(x >0 && z>0 )
	{
		if(x >z)
		{
			flag=tan_theta;
			result=upper_bound(tanv.begin(),tanv.end(),flag);
			if(result==tanv.end())
			{
				offset=sectionSize-1;
			}
			else
			{
				offset=result-tanv.begin();
			}
		}
		else
		{
			flag=1/tan_theta;
			result=upper_bound(tanv.begin(),tanv.end(),flag);
			if(result==tanv.end())
			{
				offset=sectionSize;
			}
			else
			{
				diff=result-tanv.begin();
				offset=sectionSize*2-1-(diff);
			}

		}
	}

	else if(x<0 && z>0)
	{
		if(-x>z)
		{
			flag=-tan_theta;
			result=upper_bound(tanv.begin(),tanv.end(),flag);
			if(result==tanv.end())
			{
				offset=sectionSize*3;
			}
			else
			{
				offset=sectionSize*4-1-(result-tanv.begin());
			}

		}
		else
		{
			flag=1/-tan_theta;
			result=upper_bound(tanv.begin(),tanv.end(),flag);
			if(result==tanv.end())
			{
				offset=sectionSize*3-1;
			}
			else
			{
				offset=sectionSize*2+(result-tanv.begin());
			}
		}
	}

	else if(x<0&&z<0)
	{
		if(-x>-z)
		{
			flag=tan_theta;
			result=upper_bound(tanv.begin(),tanv.end(),flag);
			if(result==tanv.end())
			{
				offset=sectionSize*5-1;
			}
			else
			{
				offset=sectionSize*4+(result-tanv.begin());
			}
		}
		else
		{
			flag=1/tan_theta;
			result=upper_bound(tanv.begin(),tanv.end(),flag);
			if(result==tanv.end())
			{
				offset=sectionSize*5;
			}
			else
			{
				offset=sectionSize*6-1-(result-tanv.begin());
			}

		}
	}

	else if(x>0&&z<0)
	{
		if(x>-z)
		{
			flag=-tan_theta;
			result=upper_bound(tanv.begin(),tanv.end(),flag);
			if(result==tanv.end())
			{
				offset=sectionSize*7;
			}
			else
			{
				offset=sectionSize*8-1-(result-tanv.begin());
			}
		}
		else
		{
			flag=1/-tan_theta;
			result=upper_bound(tanv.begin(),tanv.end(),flag);
			if(result==tanv.end())
			{
				offset=sectionSize*7-1;
			}
			else
			{
				offset=sectionSize*6+(result-tanv.begin());
			}
		}
	}

	int k= (int)((rad-MinRad)/(cellSize*1.0));
	* column=offset;
	* row=k;
	return 0;
}


int IntersectionDetection::CalCellMinDis()
{
	for(int i=0;i<cellNum;i++)
	{
		float halfCellAngle=180.0/columnNum;
		float minCellDis=2*( i*cellSize *sin(halfCellAngle));

		if(minCellDis>cellSize)
			minCellDis=cellSize;
		minCellDisList.push_back(minCellDis);
	}
	minCellDisList[0]=minCellDisList[1];

	return 0;
}


int IntersectionDetection::TransferToCellArray()
{

	int i,j,count=0;
	float flag;
	int offset;
	int sectionSize=columnNum/8;


	cellArray_AfterRegstn.resize(columnNum);
	for(i=0; i<columnNum; ++i)
		cellArray_AfterRegstn[i].resize(cellNum);

	float inc=(M_PI*2)/columnNum;

	vector<float> tanv;
	vector<float>::iterator result;

	for(i=0; i<sectionSize; ++i)
	{
		tanv.push_back(tan(inc*i));
	}

	int diff;
	for(i=0; i< allPoints_AfterRegstn.size(); ++i)
	{
		count++;
		Point  &pt= allPoints_AfterRegstn[i];

		if(pt.rad <=MinRad || pt.rad>=MaxRad)
			continue;

		if(pt.x >0 && pt.z>0 )
		{
			if(pt.x > pt.z)
			{
				flag=pt.tan_theta;
				result=upper_bound(tanv.begin(),tanv.end(),flag);
				if(result==tanv.end())
				{
					offset=sectionSize-1;
				}
				else
				{
					offset=result-tanv.begin();
				}
			}
			else
			{
				flag=1/pt.tan_theta;
				result=upper_bound(tanv.begin(),tanv.end(),flag);
				if(result==tanv.end())
				{
					offset=sectionSize;
				}
				else
				{
					diff=result-tanv.begin();
					offset=sectionSize*2-1-(diff);
				}

			}
		}

		else if(pt.x<0 && pt.z>0)
		{
			if(-pt.x>pt.z)
			{
				flag=-pt.tan_theta;
				result=upper_bound(tanv.begin(),tanv.end(),flag);
				if(result==tanv.end())
				{
					offset=sectionSize*3;
				}
				else
				{
					offset=sectionSize*4-1-(result-tanv.begin());
				}

			}
			else
			{
				flag=1/-pt.tan_theta;
				result=upper_bound(tanv.begin(),tanv.end(),flag);
				if(result==tanv.end())
				{
					offset=sectionSize*3-1;
				}
				else
				{
					offset=sectionSize*2+(result-tanv.begin());
				}
			}
		}

		else if(pt.x<0&&pt.z<0)
		{
			if(-pt.x>-pt.z)
			{
				flag=pt.tan_theta;
				result=upper_bound(tanv.begin(),tanv.end(),flag);
				if(result==tanv.end())
				{
					offset=sectionSize*5-1;
				}
				else
				{
					offset=sectionSize*4+(result-tanv.begin());
				}
			}
			else
			{
				flag=1/pt.tan_theta;
				result=upper_bound(tanv.begin(),tanv.end(),flag);
				if(result==tanv.end())
				{
					offset=sectionSize*5;
				}
				else
				{
					offset=sectionSize*6-1-(result-tanv.begin());
				}

			}
		}

		else if(pt.x>0&&pt.z<0)
		{
			if(pt.x>-pt.z)
			{
				flag=-pt.tan_theta;
				result=upper_bound(tanv.begin(),tanv.end(),flag);
				if(result==tanv.end())
				{
					offset=sectionSize*7;
				}
				else
				{
					offset=sectionSize*8-1-(result-tanv.begin());
				}
			}
			else
			{
				flag=1/-pt.tan_theta;
				result=upper_bound(tanv.begin(),tanv.end(),flag);
				if(result==tanv.end())
				{
					offset=sectionSize*7-1;
				}
				else
				{
					offset=sectionSize*6+(result-tanv.begin());
				}
			}
		}

		else
		{
			continue;
		}

		int k= (int)((pt.rad-MinRad)/(cellSize*1.0));
		cellArray_AfterRegstn[offset][k].push_back(&pt);
	}
	return 0;
}


int IntersectionDetection:: CalcCellFeature(cell& cellobj, cellFeature& f)
{
	int outlier;
	float lastMaxY;
	f.size=cellobj.size();
	f.cellType=0;

	if(f.size==0)
	{
		f.cellType|=CELL_TYPE_INVALID;
		return 0;
	}

	f.ave_x= f.ave_y = f.ave_z=0.0;
	f.delta_y=0;

	int i=0;
	for(i=0; i<f.size; ++i)
	{
		f.ave_x+=cellobj[i]->x;
		f.ave_z+=cellobj[i]->z;
		f.ave_y+=cellobj[i]->y;

		if(i==0)
		{
			outlier=0;
			f.min_x=f.max_x=cellobj[i]->x;
			f.min_z=f.max_z=cellobj[i]->z;
			lastMaxY=f.min_y=f.max_y=cellobj[i]->y;
		}
		else
		{
			if(f.max_x<cellobj[i]->x)		f.max_x=cellobj[i]->x;

			if(f.min_x>cellobj[i]->x)	   f.min_x=cellobj[i]->x;

			if(f.max_z<cellobj[i]->z)	   f.max_z=cellobj[i]->z;

			if(f.min_z>cellobj[i]->z)    f.min_z=cellobj[i]->z;

			if(f.max_y<cellobj[i]->y)
			{
				lastMaxY=f.max_y;
				f.max_y=cellobj[i]->y;
				outlier=i;
			}

			if(f.min_y>cellobj[i]->y)
				f.min_y=cellobj[i]->y;
		}
	}

	if(f.size>1)
	{
		int y=f.ave_y-cellobj[outlier]->y;
		y/=(f.size-1)*1.0;

		if(cellobj[outlier]->y-y<50)
		{
			outlier=-1;
			f.ave_y/=f.size*1.0;
		}
		else
		{
			f.max_y=lastMaxY;
			f.ave_y=y;
		}
	}
	else
	{
		outlier=-1;
		f.ave_y/=f.size*1.0;
	}

	f.ave_x/=f.size*1.0;
	f.ave_z/=f.size*1.0;

	for(i=0; i<f.size; ++i)
	{
		if(i==outlier)
			continue;
		f.delta_y+= absf(cellobj[i]->y - f.ave_y);
	}

	float threshold;
	threshold=f.delta_y;

	float GridThresholdGroundDetect =120;
	if(threshold >  GridThresholdGroundDetect)
		f.cellType|=CELL_TYPE_ABOVE_DELTA_Y;
	else
		f.cellType|=CELL_TYPE_BELOW_DELTA_Y;
	return 0;
}


int IntersectionDetection::CalcAllCellFeature()
{
	int i,j;

	if( cellArray_AfterRegstn.size()==0)
		return -1;

	if( cellFeatureArray_AfterRegstn.size()==0)
	{
		cellFeatureArray_AfterRegstn.resize(columnNum);
		for(i=0; i<columnNum; ++i)
			cellFeatureArray_AfterRegstn[i].resize(cellNum);
	}

	for(j=0; j <columnNum; j++)
	{
		cellColumn &column=cellArray_AfterRegstn[j];
		cellNum=column.size();
		for( i=0; i<cellNum; i++)
		{
			cell &cellObj=cellArray_AfterRegstn[j][i];
			cellFeature &feature=cellFeatureArray_AfterRegstn[j][i];

			feature.columnID=j;
			feature.cellID=i;

			feature.pCell=&cellObj;
			CalcCellFeature(cellObj,feature);

			//      if( feature.delta_y > 120)
			// cellFeatureArray_AfterRegstn[j][i].cellType |= CELL_TYPE_ABOVE_DELTA_Y;
		}
	}
	return 0;

}

int IntersectionDetection::CalWideSlashEdge_For_RoadShape(float Angle,int startColumID,int startCellID,float maxLength,float wide)
{
	float LengthX;
	float LengthZ;
	double sinAngle=sin(Angle*M_PI/180);
	double cosAngle=cos(Angle*M_PI/180);
	float columnAngle=360.0/columnNum;

	float XPos=cellSize*(startCellID+0.5)*cos( columnAngle*(startColumID+0.5)*M_PI/180);
	float ZPos=cellSize*(startCellID+0.5)*sin( columnAngle*(startColumID+0.5)*M_PI/180);

	float halfWide=wide/2;
	float vexPointXPos1,vexPointZPos1,vexPointXPos2,vexPointZPos2,closePointX,closePointZ;
	float moveInterval=0;
	float midPointX,midPointZ;
	int pointColumn,pointRow;

	IntersectionFeature temp;
	temp.slashLength=maxLength;
	temp.angle=Angle;

	for(float k=0;k<maxLength;k+=moveInterval)
	{
		XPos+=sinAngle*moveInterval;
		ZPos+=cosAngle*moveInterval;
		vexPointXPos1=XPos+halfWide*cosAngle;
		vexPointZPos1=ZPos-halfWide*sinAngle;
		vexPointXPos2=XPos-halfWide*cosAngle;
		vexPointZPos2=ZPos+halfWide*sinAngle;

		if(vexPointXPos1*vexPointXPos1+vexPointZPos1*vexPointZPos1>
			vexPointXPos2*vexPointXPos2+vexPointZPos2*vexPointZPos2)
		{
			closePointX=vexPointXPos2;
			closePointZ=vexPointZPos2;
		}
		else
		{
			closePointX=vexPointXPos1;
			closePointZ=vexPointZPos1;
		}
		CalPointCellPos(closePointX,0,closePointZ,&pointColumn,&pointRow);
		moveInterval=minCellDisList[pointRow];

		for(float i=0;i<=wide;i+=moveInterval)
		{
			midPointX=vexPointXPos1-i*cosAngle*moveInterval;
			midPointZ=vexPointZPos1+i*sinAngle*moveInterval;
			CalPointCellPos(midPointX,0,midPointZ,&pointColumn,&pointRow);

			if(pointRow>cellNum-1)
			{
				temp.slashEndColumn=pointColumn;
				temp.slashEndRow=cellNum-1;
				goto labelCalSlashEdge_For_RoadShape;
			}
			cellFeature& cellobj=cellFeatureArray_AfterRegstn[pointColumn][pointRow];
			if(cellobj.cellType & CELL_TYPE_ABOVE_DELTA_Y )
			{
				temp.slashEndColumn=pointColumn;
				temp.slashEndRow=pointRow;
				temp.slashLength=k;
				goto labelCalSlashEdge_For_RoadShape;

			}
		}

	}

labelCalSlashEdge_For_RoadShape:
	temp.slashLength=temp.slashLength/100;//单位化为m
	intersectionFeature.push_back(temp);

	return 0;
}


int IntersectionDetection::DetectIntersection()
{
	int startColumn=0;
	int startRow=30;

	for(float i=0;i<360;i+=slashAngleDefinition)
	{
		CalWideSlashEdge_For_RoadShape(i,startColumn,startRow,slashMaxLength,slashWide);
	}

	double labelSVM;
	for(int i=0;i<360;i++)
	{
		nod[i].index=i+1;
		nod[i].value=intersectionFeature[i].slashLength/slashMaxLength;
	}
	nod[360].index=-1;
	labelSVM= svm_predict(m,nod);

	ofstream output;
	output.open("intersection.txt");
	output<<"labelSVM:"<<labelSVM<<endl;
	for(int j=0;j<360;j++)
		output<<j<<":"<<"  "<<nod[j].value;
	output.close();

	if(labelSVM>0.5)
		cout<<"intersection"<<endl;
	else
		cout<<"segment"<<endl;

	return 0;
}

