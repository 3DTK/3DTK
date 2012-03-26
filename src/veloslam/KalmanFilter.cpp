/***************By yuanjun**************************/
#include "veloslam/KalmanFilter.h"
#include "math.h"
#define delta_t 0.1


KalmanFilter::KalmanFilter()
{
			
}

KalmanFilter::KalmanFilter(clusterFeature &glu,double rollAngle)
{
	X1.SetMatrixRowAndCol(4,1);
	X2.SetMatrixRowAndCol(4,1);
	A.SetMatrixRowAndCol(4,4);
	H.SetMatrixRowAndCol(2,4);
	Z.SetMatrixRowAndCol(2,1);
	K.SetMatrixRowAndCol(4,2);
	Q.SetMatrixRowAndCol(4,4);
	R.SetMatrixRowAndCol(2,2);
	P1.SetMatrixRowAndCol(4,4);
	P2.SetMatrixRowAndCol(4,4);
	CoordinateRoll.SetMatrixRowAndCol(4,4);

	X1.m_pTMatrix[0][0]=glu.avg_x;
	X1.m_pTMatrix[1][0]=glu.avg_z;
	X1.m_pTMatrix[2][0]=X1.m_pTMatrix[3][0]=25;


	A.Eye();
	A.m_pTMatrix[0][2]=delta_t;
	A.m_pTMatrix[1][3]=delta_t;

	H.m_pTMatrix[0][0]=1;
	H.m_pTMatrix[1][1]=1;	

	Q.Eye();

	R.Eye();

	P1.Eye();

	angle=rollAngle;

	CalCoorRoll(angle);
}

KalmanFilter::~KalmanFilter(void)
{

}

void KalmanFilter::InitialKalmanFilter(clusterFeature &glu)
{
	X1.SetMatrixRowAndCol(4,1);
	X2.SetMatrixRowAndCol(4,1);
	A.SetMatrixRowAndCol(4,4);
	H.SetMatrixRowAndCol(2,4);
	Z.SetMatrixRowAndCol(2,1);
	K.SetMatrixRowAndCol(4,2);
	Q.SetMatrixRowAndCol(4,4);
	R.SetMatrixRowAndCol(2,2);
	P1.SetMatrixRowAndCol(4,4);
	P2.SetMatrixRowAndCol(4,4);

	X1.m_pTMatrix[0][0]=glu.avg_x;
	X1.m_pTMatrix[1][0]=glu.avg_z;
	X1.m_pTMatrix[2][0]=X1.m_pTMatrix[3][0]=1;

	A.Eye();
	A.m_pTMatrix[0][2]=delta_t;
	A.m_pTMatrix[1][3]=delta_t;

	H.m_pTMatrix[0][0]=1;
	H.m_pTMatrix[1][1]=1;	

	Q.Eye();

	R.Eye();

	P1.Eye();

}

void KalmanFilter::timeUpdate()
{
	X2=A*X1;
	P2=A*P1*A.Transpose()+Q;
}

void KalmanFilter::stateUpdate(clusterFeature &glu,double rollangle)
{
	Z.m_pTMatrix[0][0]=glu.avg_x;
	Z.m_pTMatrix[1][0]=glu.avg_z;

	CMatrix temp(2,2);
	temp=H*P2*H.Transpose()+R;

	K=P2*H.Transpose()*temp.Inverse();

	CalCoorRoll(rollangle);

	X2=CoordinateRoll*X2;

	X1=X2+K*(Z-H*X2);

	CMatrix I(4,4);
	I.Eye();

	P1=(I-K*H)*P2;
}

ObjectState KalmanFilter::GetCurrentState()
{
	ObjectState movestate;
	movestate.x_position=X1.m_pTMatrix[0][0];
	movestate.z_positon=X1.m_pTMatrix[1][0];
	movestate.x_speed=X1.m_pTMatrix[2][0];
	movestate.z_speed=X1.m_pTMatrix[3][0];

	return movestate;
}

ObjectState KalmanFilter::GetPredictState()
{
	ObjectState movestate;

	movestate.x_position=X2.m_pTMatrix[0][0];
	movestate.z_positon=X2.m_pTMatrix[1][0];
	movestate.x_speed=X2.m_pTMatrix[2][0];
	movestate.z_speed=X2.m_pTMatrix[3][0];

	return movestate;
}

Measurement KalmanFilter::GetPredictMeasurement(double rollAngle)
{
	Measurement predictMeasurement;
	CMatrix temp(2,1);
	CalCoorRoll(rollAngle);
	X2=CoordinateRoll*X2;
	temp=H*X2; 
	predictMeasurement.x_measurement=temp.m_pTMatrix[0][0];
	predictMeasurement.z_measurement=temp.m_pTMatrix[1][0];

	return predictMeasurement;
}

CMatrix KalmanFilter::CalMeasureDeviation()
{
	CMatrix Deviation(2,2),standardDeviation(2,2);

	Deviation=H*P2*H.Transpose()+R; 

	for (int i=0;i<2;i++)
	{
		for(int j=0;j<2;j++)
		{
			standardDeviation.m_pTMatrix[i][j]=sqrt(Deviation.m_pTMatrix[i][j]);  //¼ÆËã±ê×¼Æ«²î
		}
	}

	return standardDeviation;
}

KalmanFilter& KalmanFilter::operator = (const KalmanFilter& anotherKF)
{
	A=anotherKF.A;
	H=anotherKF.H;
	X1=anotherKF.X1;
	X2=anotherKF.X2;
	K=anotherKF.K;
	Z=anotherKF.Z;
	P1=anotherKF.P1;
	P2=anotherKF.P2;
	Q=anotherKF.Q;
	R=anotherKF.R;
	CoordinateRoll=anotherKF.CoordinateRoll;
	angle=anotherKF.angle;
	return *this;
}

void KalmanFilter::CalCoorRoll(double angle)
{
	CoordinateRoll.m_pTMatrix[0][0]=cos(angle);//angle is radian??
	CoordinateRoll.m_pTMatrix[0][1]=sin(angle);
	CoordinateRoll.m_pTMatrix[1][0]=-sin(angle);
	CoordinateRoll.m_pTMatrix[1][1]=cos(angle);
	CoordinateRoll.m_pTMatrix[2][2]=cos(angle);
	CoordinateRoll.m_pTMatrix[2][3]=sin(angle);
	CoordinateRoll.m_pTMatrix[3][2]=-sin(angle);
	CoordinateRoll.m_pTMatrix[3][3]=cos(angle);
}

