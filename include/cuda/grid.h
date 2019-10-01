#ifndef __GRID_H__
#define __GRID_H__

#include "cuda/gridparams.h"
#include <vector>

/**
 * @brief .....
 *
 * ....
 **/
class CuGrid
{
public:
	CuGrid(int cuda_device);
	virtual ~CuGrid();
	unsigned int GetMsize();
	unsigned int GetDsize();
	double* GetM();
	double* GetD();
	void SetM(double *m_xyz, int numpoints);
	void SetD(double *d_xyz, int numpoints);
	void SetRadius(double radius);
	std::vector<int> fixedRangeSearch();

	GridParams params;

	void TransformM(double *mat);

private:
	double *d_m_xyz;
	double *d_d_xyz;

	int msize;
	int dsize;

	int device;

};

#endif
