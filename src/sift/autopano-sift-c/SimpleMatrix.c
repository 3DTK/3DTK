
/* autopano-sift, Automatic panorama image creation
 * Copyright (C) 2004 -- Sebastian Nowozin
 *
 * This program is free software released under the GNU General Public
 * License, which is included in this software package (doc/LICENSE).
 */

/* SimpleMatrix.cs
 *
 * Minimal two-dimensional matrix class implementation.
 *
 * (C) Copyright 2004 -- Sebastian Nowozin (nowozin@cs.tu-berlin.de)
 */

#include "AutoPanoSift.h"

SimpleMatrix* SimpleMatrix_new0 ()
{
	SimpleMatrix* self = (SimpleMatrix*)malloc(sizeof(SimpleMatrix));
	self->values = NULL;
	return self;
}

void SimpleMatrix_delete(SimpleMatrix* self)
{
	if (self) {
		DoubleMap_delete(self->values);
		self->values = NULL;
		free(self);
	}
}

void SimpleMatrix_init (SimpleMatrix* self, int yDim, int xDim)
{
	self->xDim = xDim;
	self->yDim = yDim;
	self->values = DoubleMap_new(yDim, xDim);
}

SimpleMatrix* SimpleMatrix_new (int yDim, int xDim)
{
	SimpleMatrix* self = SimpleMatrix_new0();
	self->xDim = xDim;
	self->yDim = yDim;
	self->values = DoubleMap_new(yDim, xDim);
	return self;
}

SimpleMatrix* SimpleMatrix_clone (SimpleMatrix* self)
{
	SimpleMatrix* cp = SimpleMatrix_new (self->yDim, self->xDim);
	
	int x, y;
	for ( y = 0 ; y < self->yDim ; ++y) {
	    for ( x = 0 ; x < self->xDim ; ++x) {
		cp->values[y][x] = self->values[y][x];
	    }
	}
	
	return (cp);
}

double SimpleMatrix_GetValue(SimpleMatrix* self, int y, int x) 
{
	return (self->values[y][x]);
}
void SimpleMatrix_SetValue(SimpleMatrix* self, int y, int x, double value )
{
	self->values[y][x] = value;
}

SimpleMatrix* SimpleMatrix_Mul(SimpleMatrix* m1, SimpleMatrix* m2)
{
	if (m1->xDim != m2->yDim)
		FatalError ("Matrixes cannot be multiplied, dimension mismatch");

	// vanilla!
	SimpleMatrix* res = SimpleMatrix_new (m1->yDim, m2->xDim);
	int y;
	for ( y = 0 ; y < m1->yDim ; ++y) {
		int x;
		for ( x = 0 ; x < m2->xDim ; ++x) {
			int k;
			for ( k = 0 ; k < m2->yDim ; ++k)
				res->values[y][x] += m1->values[y][k] * m2->values[k][x];
		}
	}
	
	return (res);
}

double SimpleMatrix_Dot (SimpleMatrix* self, SimpleMatrix* m)
{
	if (self->yDim != m->yDim || self->xDim != 1 || m->xDim != 1)
		FatalError
			("Dotproduct only possible for two equal n x 1 matrices");

	double sum = 0.0;

	int y;
	for ( y = 0 ; y < self->yDim ; ++y)
		sum += self->values[y][0] * m->values[y][0];

	return (sum);
}

void SimpleMatrix_Negate (SimpleMatrix* self)
{
	int y;
	for (y = 0 ; y < self->yDim ; ++y) {
		int x;
		for ( x = 0 ; x < self->xDim ; ++x) {
			self->values[y][x] = -self->values[y][x];
		}
	}
}

void SimpleMatrix_Inverse (SimpleMatrix* self)
{
	if (self->xDim != self->yDim)
		FatalError("Matrix x dimension != y dimension");

	// Shipley-Coleman inversion, from
	// http://www.geocities.com/SiliconValley/Lab/4223/fault/ach03.html
	int dim = self->xDim;
	int i,j,k;
	for ( k = 0 ; k < dim ; ++k) {
		self->values[k][k] = - 1.0 / self->values[k][k];

		for ( i = 0 ; i < dim ; ++i) {
			if (i != k)
				self->values[i][k] *= self->values[k][k];
		}

		for ( i = 0 ; i < dim ; ++i) {
			if (i != k) {
				for ( j = 0 ; j < dim ; ++j) {
					if (j != k)
						self->values[i][j] += self->values[i][k] * self->values[k][j];
				}
			}
		}
		
		for ( i = 0 ; i < dim ; ++i) {
			if (i != k)
				self->values[k][i] *= self->values[k][k];
		}
		
	}
	
	for ( i = 0 ; i < dim ; ++i) {
		for ( j = 0 ; j < dim ; ++j)
			self->values[i][j] = -self->values[i][j];
	}
}

// The vector 'vec' is used both for input/output purposes. As input, it
// contains the vector v, and after this method finishes it contains x,
// the solution in the formula
//    self * x = v
// This matrix might get row-swapped, too.
void SimpleMatrix_SolveLinear (SimpleMatrix* self, SimpleMatrix* vec)
{
	if (self->xDim != self->yDim || self->yDim != vec->yDim)
		FatalError ("Matrix not quadratic or vector dimension mismatch");
	
	// Gaussian Elimination Algorithm, as described by
	// "Numerical Methods - A Software Approach", R.L. Johnston
	
	// Forward elimination with partial pivoting
	int x, y;
	for (y = 0 ; y < (self->yDim - 1) ; ++y) {
		
		// Searching for the largest pivot (to get "multipliers < 1.0 to
		// minimize round-off errors")
		int yMaxIndex = y;
		double yMaxValue = abs (self->values[y][y]);
		
		int py;
		for (py = y ; py < self->yDim ; ++py) {
			if (abs (self->values[py][y]) > yMaxValue) {
				yMaxValue = abs (self->values[py][y]);
				yMaxIndex = py;
			}
		}

		// if a larger row has been found, swap with the current one
		SimpleMatrix_SwapRow (self, y, yMaxIndex);
		SimpleMatrix_SwapRow (vec, y, yMaxIndex);
		
		// Now do the elimination left of the diagonal
		for ( py = y + 1 ; py < self->yDim ; ++py) {
			// always <= 1.0
			double elimMul = self->values[py][y] / self->values[y][y];
			
			for ( x = 0 ; x < self->xDim ; ++x)
				self->values[py][x] -= elimMul * self->values[y][x];
			
			// FIXME: do we really need this?
			vec->values[py][0] -= elimMul * vec->values[y][0];
		}
	}
	
	// Back substitution
	for ( y = self->yDim - 1 ; y >= 0 ; --y) {
		double solY = vec->values[y][0];
		
		for ( x = self->xDim - 1 ; x > y ; --x)
			solY -= self->values[y][x] * vec->values[x][0];
		
		vec->values[y][0] = solY / self->values[y][y];
	}
}

// Swap two rows r1, r2
void SimpleMatrix_SwapRow (SimpleMatrix* self, int r1, int r2)
{
	if (r1 == r2)
		return;
	
	int x;
	for ( x = 0 ; x < self->xDim ; ++x) {
		double temp = self->values[r1][x];
		self->values[r1][x] = self->values[r2][x];
		self->values[r2][x] = temp;
	}
}

char* SimpleMatrix_ToString (SimpleMatrix* self)
{
	int len = 5 + self->yDim * (3 + self->xDim*15);
	char* str = (char*)malloc(len);
	char* p = str;

	p += sprintf(p, "( ");
	int x,y;
	for ( y = 0 ; y < self->yDim ; ++y) {
		if (y > 0)
			p += sprintf(p, "\n  ");
		
		for ( x = 0 ; x < self->xDim ; ++x) {
			if (x > 0)
				p += sprintf(p, "  ");
			
			p += sprintf(p, "%3.015g", self->values[y][x]);
		}
	}
	p+= sprintf(p, " )");
	if (p > str + len) {
		FatalError("SimpleMatrix_toString overflow: len=%d, strlen=%d\n", len, p-str);
	}

	return str;
}


#ifdef TEST_MAIN
int main(int argc, char* argv[]) {
    SimpleMatrix * A = SimpleMatrix_new(3,3);
    A->values[0][0] = 1;
    A->values[0][1] = 2;
    A->values[0][2] = 3;
    A->values[1][0] = 4;
    A->values[1][1] = 5;
    A->values[1][2] = 6;
    A->values[2][0] = 7;
    A->values[2][1] = 9;
    A->values[2][2] = 8;

    WriteLine("A=%s", SimpleMatrix_ToString(A));

    SimpleMatrix * B = SimpleMatrix_new(3,1);
    B->values[0][0] = 1;
    B->values[1][0] = 2;
    B->values[2][0] = 3;

    WriteLine("B=%s", SimpleMatrix_ToString(B));
    SimpleMatrix * C = SimpleMatrix_Mul(A,B);

    WriteLine("A*B=%s", SimpleMatrix_ToString(C));

    SimpleMatrix_delete(A);
    SimpleMatrix_delete(B);
    SimpleMatrix_delete(C);
    return 0;
}
#endif
