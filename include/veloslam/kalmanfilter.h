/***************By yuanjun**************************/
#pragma once

#include "veloslam/veloscan.h"
#include "veloslam/matrix.h"

using namespace std;

typedef struct _X_state
{
	float x_position;
	float z_positon;
	float x_speed;
	float z_speed;

}ObjectState;

typedef struct _Z_measurement
{
	float x_measurement;
	float z_measurement;
}Measurement;

class KalmanFilter
{
public:
	KalmanFilter();
	KalmanFilter(clusterFeature &glu,double rollAngle);
	~KalmanFilter(void);

	void InitialKalmanFilter(clusterFeature &glu);

	void timeUpdate();

	void stateUpdate(clusterFeature &glu,double rollAngle,double *pos);

	ObjectState GetCurrentState();

	ObjectState GetPredictState();

	Measurement GetPredictMeasurement(double rollAngle,double *pos);

	CMatrix CalMeasureDeviation();

	KalmanFilter& operator = (const KalmanFilter& anotherKF);

	void CalCoorRoll(double angle);

public:

    CMatrix A;

    CMatrix H;

	CMatrix X1;

	CMatrix X2;

	CMatrix Z;

	CMatrix K;

	CMatrix Q;

	CMatrix R;

	CMatrix P1;

	CMatrix P2;

	CMatrix CoordinateRoll;

	double angle;
};



