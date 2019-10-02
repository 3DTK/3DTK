#ifndef _COLLSISION_H_
#define _COLLSISION_H_

void cudaTransformScan(double *destx, double *desty, double *destz, double *mat, double *srcx, double *srcy, double *srcz, unsigned int size);
void cudaTransformScan(double *destx, double *desty, double *destz, double *mat, unsigned int size);
void cudaTransformScan(double *destxyz, double *mat, double *srcxyz, unsigned int size);
void cudaTransformScan(double *destxyz, double *mat, unsigned int size);

void cudaFindNN(double *d_mxyz,
				double *d_dxyz,
				unsigned int *index_m,
				unsigned int *index_d,
				unsigned int *_d_keysReference,
				int *_d_table_of_buckets,
				int *d_num_points_in_bucket,
				int *_d_NN,
				int numpoints_m,
				int numpoints_d,
				int num_buckets,
				double *_d_temp_distances,
				double _additionalDistanceCheck);

#endif
