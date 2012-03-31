/////////////////////////////////////////////////////////////////////////////
// Matrix.h : 
//		Interface of the class CMatrix
// Author : freeia
// Modified Date : 3/11/2003
// E-mail : freeia@163.com
// Company/School : hdcad
/////////////////////////////////////////////////////////////////////////////

#ifndef _MATRIX_H
#define _MATRIX_H

#include <vector>

using namespace std;

typedef vector <double> VDOUBLE;
typedef vector <VDOUBLE> TMatrix;


class CMatrix  
{	

	/************************************************************************
	*				the interface function of the class CMatrix 			*
	************************************************************************/
public:

	/////////////////////////////////////////////////////////////////////////
	// Construction and Destruction	
	CMatrix();

	CMatrix(CMatrix& cMatrixB);

	 ~CMatrix();

	TMatrix	m_pTMatrix;				

	/////////////////////////////////////////////////////////////////////////
	// According to the parameters nRow & nCol to construct a matrix
	CMatrix(unsigned int nRow, unsigned int nCol);


	/////////////////////////////////////////////////////////////////////////
	// This function initialize the matrix :
	//		the matrix which has been initialized has 0 row & 0 column, and
	// all elements in it is zeros.
	// 
	void Initialize();

	/////////////////////////////////////////////////////////////////////////
	// This function initialize the matrix :
	// all elements in it is zeros.
	// 
	void InitializeZero();

	/////////////////////////////////////////////////////////////////////////
	// To make random in the elements of the matrix and the elements of the 
	// matrix has been randomized between -1 and 1.These elements must be
	// decimal fractions.
	// 
	void RandomInitialize();

	/////////////////////////////////////////////////////////////////////////
	// Overload Operations

	// 'CMatrix + CMatrix'
	CMatrix operator + (CMatrix& cMatrixB);	
	// 'CMatrix - CMatrix'
	CMatrix operator - (CMatrix& cMatrixB);	
	// 'CMatrix * CMatrix'
	CMatrix operator * (CMatrix& cMatrixB);	
	// 'CMatrix * double'
	CMatrix operator * (double nValue);	
	// 'CMatrix = CMatrix'
	CMatrix& operator = (CMatrix& cMatrixB);	
	CMatrix& operator = (const CMatrix& cMatrixB);
	// 'CMatrix += CMatrix'
	CMatrix& operator += (CMatrix& cMatrixB);	
	// 'CMatrix .* CMatrix'
	CMatrix operator / (CMatrix& cMatrixB);	

	CMatrix Transpose();

	/////////////////////////////////////////////////////////////////////////
	// Inverse the matrix
	//
	CMatrix Inverse();

	unsigned int GetMatrixRowNumber() const
	{
		return m_nRow;
	}

	/////////////////////////////////////////////////////////////////////////
	// Get the matrix Colum Number
	//
	unsigned int GetMatrixColNumber() const
	{
		return m_nCol;
	}

	void CopySubMatrixFromVector(CMatrix& cMatrix,unsigned int nIndex);

	void CopySubMatrix(CMatrix& cMatrix,unsigned int nStartX,unsigned int nStartY);

	void CopyMatrix(CMatrix& cMatrix);

	CMatrix MergeColumnsToColumnVector();

	CMatrix Sigmoid();

	CMatrix SigmoidDerivative();

	CMatrix tanh(); 

	CMatrix tanhDerivative();

	CMatrix Tansig();

	CMatrix TansigDerivative();

	void MakeAllColumnElementsSameValue(unsigned int nRowIndex);

	void Eye();

	double	GetSystemError() const;

	CMatrix AbsoluteValue();

	void GetMatrixData(CMatrix& cMatrix, unsigned int nIndex);

	void SetMatrixData(CMatrix& cMatrix, unsigned int nIndex);


	void GetMatrixRowData(CMatrix& cMatrix, unsigned int nIndex, unsigned int nRow);


	void SetMatrixRowData(CMatrix& cMatrix, unsigned int nIndex, unsigned int nRow);

	/////////////////////////////////////////////////////////////////////////
	// Get the total value of the matrix
	double GetTotalElementValue();


	void nncpyi(CMatrix &cMatrix, unsigned int nTimes);

						
	void nncpyd(CMatrix &cMatrix);


	void nncpy (CMatrix& cMatrix, unsigned int nTimes);



public:

	unsigned int m_nRow;			
	unsigned int m_nCol;			


	void SetMatrixRowNumber(unsigned int nRow);

	void SetMatrixColNumber(unsigned int nCol);

	void SetMatrixRowAndCol(unsigned int nRow,unsigned int nCol);

	void SwapMatrixRow(unsigned int nRow1,unsigned int nRow2);

	void SwapMatrixCol(unsigned int nCol1,unsigned int nCol2);
};


/////////////////////////////////////////////////////////////////////////////
// overload operator 'double - CMatrix'
CMatrix operator - (double nValue,CMatrix& cMatrixB);

CMatrix MergeMatrix(CMatrix& cMatrixA,CMatrix& cMatrixB);


#endif // _MATRIX_H