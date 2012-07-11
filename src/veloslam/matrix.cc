/*
 * matrix implementation
 *
 * Copyright (C) YuanJun, ZhangLiang, Li Wei, Li Ming, Andreas Nuechter,
 *
 * Released under the GPL version 3.
 *
 */

/**
 * @file
 * @brief 
 *
 * @author Andreas Nuechter. Jacobs University Bremen, Germany
 * @author YuanJun, Wuhan University, China
 * @author ZhangLiang, Wuhan University, China
 * @author Li Wei, Wuhan University, China
 * @author Li Ming, Wuhan University, China
 */

#include "veloslam/matrix.h"
#include <math.h>
#include <stdlib.h>

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

CMatrix::CMatrix()
{
	m_nRow = 0;
	m_nCol = 0;

	m_pTMatrix.resize (m_nRow);
	for(unsigned int i=0; i < m_nRow; i++)
	{
		for(unsigned int j=0; j < m_nCol; j++)
		{
			m_pTMatrix[i].resize (m_nCol);
			m_pTMatrix[i][j] = (double) 0;
		}
	}
}


CMatrix::~CMatrix()
{

}


CMatrix::CMatrix(unsigned int nRow,unsigned int nCol)
{

	TMatrix tMatrix;
	tMatrix.resize (nRow);

	for(unsigned int i=0; i < nRow; i++)
	{
		for(unsigned int j=0; j < nCol; j++)
		{
	        tMatrix[i].resize(nCol);
			tMatrix[i][j] = (double) 0;
		}
	}

	m_nRow	= nRow;
	m_nCol	= nCol;
	m_pTMatrix = tMatrix;

}


CMatrix::CMatrix(CMatrix& cMatrixB)
{
	// Initialize the variable
	m_nRow = cMatrixB.m_nRow ;
	m_nCol = cMatrixB.m_nCol ;
	m_pTMatrix = cMatrixB.m_pTMatrix ;

	// Copy Data
	for(unsigned int i=0; i< cMatrixB.m_nRow; i++)
	{
		for(unsigned int j=0; j < cMatrixB.m_nCol; j++)
		{
			m_pTMatrix [i][j] = cMatrixB.m_pTMatrix [i][j];
		}
	}

}


/////////////////////////////////////////////////////////////////////////////
// CMatrix member functions
//

CMatrix CMatrix::operator +(CMatrix& cMatrixB)
{


	CMatrix	cMatrix = *this;

	for(unsigned int i=0; i < m_nRow; i++)
	{
		for(unsigned int j=0; j < m_nCol; j++)
		{
			cMatrix.m_pTMatrix [i][j] = m_pTMatrix [i][j] + cMatrixB.m_pTMatrix [i][j];
		}
	}

	return	cMatrix;

}


CMatrix CMatrix::operator -(CMatrix& cMatrixB)
{

	CMatrix cMatrix = *this;

	for(unsigned int i=0; i < m_nRow; i++)
	{
		for(unsigned int j=0; j < m_nCol; j++)
		{
			cMatrix.m_pTMatrix [i][j] = m_pTMatrix [i][j] - cMatrixB.m_pTMatrix [i][j];
		}
	}

	return	cMatrix;

}


CMatrix CMatrix::operator *(CMatrix& cMatrixB)
{

	CMatrix cResultMatrix(m_nRow,cMatrixB.m_nCol);

	for(unsigned int i=0; i < m_nRow; i++)
	{
		for(unsigned int j=0; j < cMatrixB.m_nCol; j++)
		{
			for(unsigned int m=0; m < m_nCol; m++)
			{
				cResultMatrix.m_pTMatrix [i][j] +=  m_pTMatrix [i][m] * cMatrixB.m_pTMatrix [m][j];
			}
		}
	}

	return cResultMatrix;
}


CMatrix CMatrix::operator * (double nValue)
{
	CMatrix cMatrix = *this;

	for(unsigned int i=0; i < m_nRow; i++)
	{
		for(unsigned int j=0; j < m_nCol; j++)
		{
			cMatrix.m_pTMatrix [i][j] =m_pTMatrix [i][j] * nValue;
		}
	}

	return cMatrix;
}


CMatrix& CMatrix::operator =(CMatrix& cMatrixB)
{

	m_nRow = cMatrixB.m_nRow ;
	m_nCol = cMatrixB.m_nCol ;
	m_pTMatrix = cMatrixB.m_pTMatrix ;

	for(unsigned int i=0; i < cMatrixB.m_nRow; i++)
	{
		for(unsigned int j=0; j< cMatrixB.m_nCol; j++)
		{
			m_pTMatrix [i][j] = cMatrixB.m_pTMatrix [i][j];
		}
	}

	return *this;
}

CMatrix& CMatrix::operator =(const CMatrix& cMatrixB)
{

	m_nRow = cMatrixB.m_nRow ;
	m_nCol = cMatrixB.m_nCol ;
	m_pTMatrix = cMatrixB.m_pTMatrix ;

	for(unsigned int i=0; i < cMatrixB.m_nRow; i++)
	{
		for(unsigned int j=0; j< cMatrixB.m_nCol; j++)
		{
			m_pTMatrix [i][j] = cMatrixB.m_pTMatrix [i][j];
		}
	}

	return *this;
}

CMatrix& CMatrix::operator += (CMatrix& cMatrixB)
{

	for(unsigned int i=0; i < cMatrixB.m_nRow; i++)
	{
		for(unsigned int j=0; j< cMatrixB.m_nCol; j++)
		{
			m_pTMatrix [i][j] += cMatrixB.m_pTMatrix [i][j];
		}
	}

	return *this;

}


CMatrix CMatrix::Transpose()
{
	CMatrix cMatrix(m_nCol,m_nRow);

	for(unsigned int i=0; i < m_nRow; i++)
	{
		for(unsigned int j=0; j < m_nCol; j++)
		{
			cMatrix.m_pTMatrix [j][i] = m_pTMatrix [i][j];
		}
	}

	return cMatrix;
}



CMatrix CMatrix::MergeColumnsToColumnVector()
{
	CMatrix cMatrix(m_nRow * m_nCol,(unsigned int)1);

	for(unsigned int j=0; j < m_nCol; j++)
	{
		for(unsigned int i=0; i < m_nRow; i++)
		{
			cMatrix.m_pTMatrix [i + j * m_nRow][(unsigned int)0] = m_pTMatrix [i][j];
		}
	}

	return cMatrix;

}

/////////////////////////////////////////////////////////////////////////////
// Get the total value of the matrix
/////////////////////////////////////////////////////////////////////////////

double CMatrix::GetTotalElementValue()
{
	double	nTotalValue = 0;

	for(unsigned int i=0; i < m_nRow; i++)
	{
		for( unsigned int j=0; j < m_nCol; j++)
		{
			nTotalValue += m_pTMatrix [i][j];
		}
	}

	return nTotalValue;
}

/////////////////////////////////////////////////////////////////////////////
// Get System Error
/////////////////////////////////////////////////////////////////////////////

double	CMatrix::GetSystemError() const
{
	double	nSystemError = 0;

	for(unsigned int i=0; i < m_nRow; i++)
	{
		for( unsigned int j=0; j < m_nCol; j++)
		{
			nSystemError += m_pTMatrix [i][j] * m_pTMatrix [i][j];
		}
	}

	return nSystemError;

}

/////////////////////////////////////////////////////////////////////////////
// Make all the matrix elements to be changed into absolute value
/////////////////////////////////////////////////////////////////////////////

CMatrix CMatrix::AbsoluteValue ()
{
	CMatrix cMatrix = *this;

	for(unsigned int i=0; i < m_nRow; i++)
	{
		for(unsigned int j=0; j < m_nCol; j++)
		{
			cMatrix.m_pTMatrix [i][j] = fabs( m_pTMatrix [i][j]);

		}

	}

	return cMatrix;

}


CMatrix CMatrix::Inverse()
{


	CMatrix cMatrix = *this;

	int *pIntArray = new int [2*m_nCol];

	for(unsigned int k=0; k < cMatrix.m_nCol; k++)
	{

		double nMaxElement = cMatrix.m_pTMatrix [k][k];
		unsigned int nMainRow = k;

		for(unsigned int nCount = k+1; nCount < cMatrix.m_nCol; nCount++)
		{
			if( fabs(nMaxElement) < fabs(cMatrix.m_pTMatrix [nCount][k]) )
			{
				nMaxElement = cMatrix.m_pTMatrix [nCount][k];
				nMainRow = nCount;
			}
		}

		pIntArray [2*k] = k;
		pIntArray [2*k+1] = nMainRow;

		cMatrix.SwapMatrixRow(k,nMainRow);


		cMatrix.m_pTMatrix [k][k] = 1/(cMatrix.m_pTMatrix [k][k]);

		for(unsigned int i=0; i < cMatrix.m_nRow; i++)
		{
			if( i != k)
				cMatrix.m_pTMatrix [i][k] = -(cMatrix.m_pTMatrix [k][k]) * (cMatrix.m_pTMatrix [i][k]);

		}


		for(unsigned int m=0; m < cMatrix.m_nRow; m++)
		{
			if ( m == k)
				continue;

			for(unsigned int n=0; n < cMatrix.m_nCol; n++)
			{
				if ( n == k)
					continue;

				cMatrix.m_pTMatrix [m][n] += cMatrix.m_pTMatrix [m][k] * cMatrix.m_pTMatrix [k][n];

			}


		}

		for(unsigned int j=0; j < cMatrix.m_nCol; j++)
		{
			if( j != k)
				cMatrix.m_pTMatrix [k][j] = (cMatrix.m_pTMatrix [k][k]) * (cMatrix.m_pTMatrix [k][j]);

		}

	}



	for(int i=2*m_nCol-1; i > 0; i--)
	{
		cMatrix.SwapMatrixCol(pIntArray[i],pIntArray[i-1]);
		i--;
	}

	delete []pIntArray;

	return cMatrix;

}


void CMatrix::SwapMatrixRow(unsigned int nRow1,unsigned int nRow2)
{
	if( nRow1 == nRow2)
		return;

	double *pArray = new double;

	for(unsigned int i=0; i < m_nCol; i++)
	{
		// Swap the datum of the two rows
		pArray[0] = m_pTMatrix [nRow1][i];
		m_pTMatrix [nRow1][i] = m_pTMatrix [nRow2][i];
		m_pTMatrix [nRow2][i] = pArray[0];
	}

	delete pArray;
}


void CMatrix::SwapMatrixCol(unsigned int nCol1,unsigned int nCol2)
{
	if( nCol1 == nCol2)
		return;

	double *pArray = new double;
	for(unsigned int i=0; i < m_nRow; i++)
	{
		// Swap the datum of the two columns
		pArray[0] = m_pTMatrix [i][nCol1];
		m_pTMatrix [i][nCol1] = m_pTMatrix [i][nCol2];
		m_pTMatrix [i][nCol2] = pArray[0];
	}

	delete pArray;
}




void CMatrix::Eye()
{


	for(unsigned int i=0; i < m_nRow; i++)
	{
		for(unsigned int j=0; j < m_nCol; j++)
		{
			if(i == j)
			{
				m_pTMatrix [i][j] =	1;
			}
			else
			{
				m_pTMatrix [i][j] =	0;
			}
		}

	}


}




void CMatrix::GetMatrixData(CMatrix& cMatrix, unsigned int nIndex)
{


	for(unsigned int i=0; i < cMatrix.m_nRow; i++)
	{
		for(unsigned int j=0; j < cMatrix.m_nCol; j++)
		{
			m_pTMatrix [nIndex + i * cMatrix.m_nCol + j][0] = cMatrix.m_pTMatrix [i][j];
		}

	}

}


void CMatrix::SetMatrixData(CMatrix& cMatrix, unsigned int nIndex)
{

	for(unsigned int i=0; i < cMatrix.m_nRow; i++)
	{
		for(unsigned int j=0; j < cMatrix.m_nCol; j++)
		{
			cMatrix.m_pTMatrix [i][j] = m_pTMatrix [nIndex + i * cMatrix.m_nCol + j][0];


		}
	}

}




void CMatrix::SetMatrixRowData(CMatrix& cMatrix, unsigned int nIndex, unsigned int nRow)
{

	for(unsigned int i=0; i < cMatrix.m_nCol; i++)
	{
		cMatrix.m_pTMatrix [nRow][i] = m_pTMatrix [nIndex + i][(unsigned int)0];
	}

}




void CMatrix::GetMatrixRowData(CMatrix& cMatrix, unsigned int nIndex, unsigned int nRow)
{


	for(unsigned int i=0; i < cMatrix.m_nCol; i++)
	{
		m_pTMatrix [nIndex + i][(unsigned int)0] = cMatrix.m_pTMatrix [nRow][i];
	}

}

void CMatrix::SetMatrixRowNumber(unsigned int nRow)
{
	m_nRow = nRow;

	m_pTMatrix.resize (m_nRow);
	for(unsigned int i=0; i < m_nRow; i++)
	{
		m_pTMatrix[i].resize (m_nCol);
		for(unsigned int j=0; j < m_nCol; j++)
		{
			m_pTMatrix[i][j] = (double) 0;
		}
	}

}


void CMatrix::SetMatrixColNumber(unsigned int nCol)
{
	m_nCol = nCol;

	m_pTMatrix.resize (m_nRow);
	for(unsigned int i=0; i < m_nRow; i++)
	{
		m_pTMatrix[i].resize (m_nCol);
		for(unsigned int j=0; j < m_nCol; j++)
		{
			m_pTMatrix[i][j] = (double) 0;
		}
	}

}

void CMatrix::SetMatrixRowAndCol(unsigned int nRow,unsigned int nCol)
{
	m_nRow = nRow;
	m_nCol = nCol;

	m_pTMatrix.resize (m_nRow);
	for(unsigned int i=0; i < m_nRow; i++)
	{
		m_pTMatrix[i].resize (m_nCol);
		for(unsigned int j=0; j < m_nCol; j++)
		{
			m_pTMatrix[i][j] = (double) 0;
		}
	}

}



void CMatrix::Initialize()
{
	m_nRow = 0;
	m_nCol = 0;

	m_pTMatrix.resize (m_nRow);
	for(unsigned int i=0; i < m_nRow; i++)
	{
		m_pTMatrix[i].resize (m_nCol);
		for(unsigned int j=0; j < m_nCol; j++)
		{
			m_pTMatrix[i][j] = (double) 0;
		}
	}

}



void CMatrix::InitializeZero()
{

	for(unsigned int i=0; i < m_nRow; i++)
	{
		for(unsigned int j=0; j < m_nCol; j++)
		{
			m_pTMatrix[i][j] = (double) 0;
		}
	}

}


void CMatrix::RandomInitialize ()
{
	for(unsigned int i=0; i < m_nRow; i++)
	{
		for(unsigned int j=0; j < m_nCol; j++)
		{
			m_pTMatrix [i][j] = (double)(rand() - (0.5*RAND_MAX)) / (0.5*RAND_MAX);
		}
	}

}



void CMatrix::CopySubMatrix(CMatrix& cMatrix,unsigned int nStartX,unsigned int nStartY)
{


	for(unsigned int i=0;  i < cMatrix.m_nRow; i++)
	{
		for(unsigned int j=0; j < cMatrix.m_nCol; j++)
		{
			cMatrix.m_pTMatrix [i][j] = m_pTMatrix [nStartY + i][nStartX + j];
		}
	}

}


void CMatrix::CopyMatrix(CMatrix& cMatrix)
{
	m_nRow	= cMatrix.m_nRow ;
	m_nCol	= cMatrix.m_nCol ;

	m_pTMatrix	= cMatrix.m_pTMatrix ;

	for(unsigned int i=0; i < m_nRow; i++)
	{
		for(unsigned int j=0; j < m_nCol; j++)
		{
			m_pTMatrix [i][j] = cMatrix.m_pTMatrix [i][j];
		}

	}

}


void CMatrix::CopySubMatrixFromVector(CMatrix& cMatrix,unsigned int nIndex)
{


	for(unsigned int j=0; j < cMatrix.m_nCol; j++)
	{
		for(unsigned int i=0; i < cMatrix.m_nRow; i++)
		{
			cMatrix.m_pTMatrix [i][j] = m_pTMatrix [nIndex + j * cMatrix.m_nRow + i ][(unsigned int)0];
		}
	}

}



void CMatrix::nncpyi(CMatrix &cMatrix, unsigned int nTimes)
{
	unsigned int i;

	m_nRow	=	cMatrix.m_nRow ;
	m_nCol	=	cMatrix.m_nCol *	nTimes;

	m_pTMatrix.resize (m_nRow);
	for(int i=0; i < m_nRow; i++)
	{
		for(unsigned int j=0; j < m_nCol; j++)
		{
			m_pTMatrix[i].resize (m_nCol);
			m_pTMatrix[i][j] = (double) 0;
		}
	}

	for(i=0; i < m_nRow; i++)
	{
		for(unsigned int j=0; j < cMatrix.m_nCol; j++)
		{
			for(unsigned int k=0; k < nTimes; k++)
			{
				m_pTMatrix [i][j * nTimes + k] = cMatrix.m_pTMatrix [i][j];
			}
		}
	}

}



void CMatrix::nncpyd(CMatrix &cMatrix)
{
	unsigned int i;
	m_nRow	=	cMatrix.m_nRow ;
	m_nCol	=	cMatrix.m_nCol * cMatrix.m_nRow ;

	m_pTMatrix.resize (m_nRow);
	for(int i=0; i < m_nRow; i++)
	{
		for(unsigned int j=0; j < m_nCol; j++)
		{
			m_pTMatrix[i].resize (m_nCol);
			m_pTMatrix[i][j] = (double) 0;
		}
	}

	for(i=0; i < m_nRow; i++)
	{
		for(unsigned int j=0; j < cMatrix.m_nCol; j++)
		{
			for(unsigned int k=0; k < cMatrix.m_nRow; k++)
			{
				if(i == (j * cMatrix.m_nRow + k) % cMatrix.m_nRow )
					m_pTMatrix [i][j * cMatrix.m_nRow + k] = cMatrix.m_pTMatrix [i][j];
			}
		}
	}

}



void CMatrix::nncpy(CMatrix& cMatrix,unsigned int nTimes)
{
	unsigned int i,j;
	m_nRow = cMatrix.m_nRow ;
	m_nCol = cMatrix.m_nCol * nTimes;

	m_pTMatrix.resize (m_nRow);
	for(int i=0; i < m_nRow; i++)
	{
		for(int j=0; j < m_nCol; j++)
		{
			m_pTMatrix[i].resize (m_nCol);
			m_pTMatrix[i][j] = (double) 0;
		}
	}

	for(i=0; i < m_nRow; i++)
	{
		for(unsigned int j=0; j < nTimes; j++)
		{
			for(unsigned int k=0; k < cMatrix.m_nCol; k++)
			{
				m_pTMatrix [i][j * cMatrix.m_nCol + k] = cMatrix.m_pTMatrix [i][k];
			}
		}
	}

}



CMatrix CMatrix::Sigmoid()
{
	CMatrix cMatrix = *this;

	for(unsigned int i=0; i < m_nRow; i++)
	{
		for(unsigned int j=0; j < m_nCol; j++)
		{
			cMatrix.m_pTMatrix [i][j] = 1 / (1 + exp(-m_pTMatrix [i][j]));
		}

	}

	return cMatrix;
}


CMatrix CMatrix::tanh ()
{
	CMatrix cMatrix = *this;

	for(unsigned int i=0; i < m_nRow; i++)
	{
		for(unsigned int j=0; j < m_nCol; j++)
		{
			cMatrix.m_pTMatrix [i][j] = 1 - (2 * exp(-m_pTMatrix [i][j])) / (1 + exp(-m_pTMatrix [i][j]));
		}

	}

	return cMatrix;
}



CMatrix CMatrix::Tansig()
{
	CMatrix cMatrix = *this;

	for(unsigned int i=0; i < m_nRow; i++)
	{
		for(unsigned int j=0; j < m_nCol; j++)
		{
			cMatrix.m_pTMatrix [i][j] = 2 / (1 + exp(- 2 * m_pTMatrix [i][j])) - 1;
		}
	}

	return cMatrix;

}



CMatrix CMatrix::TansigDerivative()
{
	CMatrix cMatrix = *this;

	for(unsigned int i=0; i < m_nRow; i++)
	{
		for(unsigned int j=0; j < m_nCol; j++)
		{
			cMatrix.m_pTMatrix [i][j] = (2 / (1 + exp(- 2 * m_pTMatrix [i][j])) - 1) * (2 / (1 + exp(- 2 * m_pTMatrix [i][j])) - 1) - 1;
		}
	}

	return cMatrix;

}


void CMatrix::MakeAllColumnElementsSameValue(unsigned int nRowIndex)
{
	for(unsigned int i=0; i < m_nRow; i++)
	{
		if(i == nRowIndex)
			continue;

		for(unsigned int j=0; j < m_nCol; j++)
		{
			m_pTMatrix [i][j] = m_pTMatrix [nRowIndex][j];
		}
	}

}



CMatrix CMatrix::SigmoidDerivative()
{
	CMatrix cMatrix = *this;

	for(unsigned int i=0; i < m_nRow; i++)
	{
		for(unsigned int j=0; j < m_nCol; j++)
		{
			cMatrix.m_pTMatrix [i][j] = exp(-m_pTMatrix [i][j]) / ((1 + exp(-m_pTMatrix [i][j])) * (1 + exp(-m_pTMatrix [i][j])));
		}

	}

	return cMatrix;
}



CMatrix CMatrix::tanhDerivative()
{
	CMatrix cMatrix = *this;

	for(unsigned int i=0; i < m_nRow; i++)
	{
		for(unsigned int j=0; j < m_nCol; j++)
		{
			cMatrix.m_pTMatrix [i][j] = 2 * exp(-m_pTMatrix [i][j]) / ((1 + exp(-m_pTMatrix [i][j])) * (1 + exp(-m_pTMatrix [i][j])));
		}

	}

	return cMatrix;
}



CMatrix CMatrix::operator / (CMatrix& cMatrixB)
{
	CMatrix cMatrix = *this;


	for(unsigned int i=0; i < m_nRow; i++)
	{
		for(unsigned int j=0; j < m_nCol; j++)
		{
			cMatrix.m_pTMatrix [i][j] = m_pTMatrix [i][j] * cMatrixB.m_pTMatrix [i][j];
		}

	}

	return cMatrix;

}





CMatrix operator - (double nValue,CMatrix& cMatrixB)
{
	CMatrix	cMatrix = cMatrixB;

	for(unsigned int i=0; i < cMatrix.GetMatrixRowNumber (); i++)
	{
		for(unsigned int j=0; j < cMatrix.GetMatrixColNumber (); j++)
		{
			cMatrix.m_pTMatrix [i][j] = nValue - cMatrixB.m_pTMatrix [i][j];
		}
	}

	return cMatrix;
}




CMatrix MergeMatrix(CMatrix& cMatrixA,CMatrix& cMatrixB)
{


	CMatrix cMatrix(cMatrixA.GetMatrixRowNumber (),cMatrixA.GetMatrixColNumber () + cMatrixB.GetMatrixColNumber ());

	for(unsigned int i=0; i < cMatrixA.GetMatrixRowNumber (); i++)
	{
		for(unsigned int j=0; j < cMatrixA.GetMatrixColNumber (); j++)
		{
			cMatrix.m_pTMatrix [i][j] = cMatrixA.m_pTMatrix [i][j];
		}

		for(unsigned int k=0; k < cMatrixB.GetMatrixColNumber (); k++)
		{
			cMatrix.m_pTMatrix [i][cMatrixA.GetMatrixColNumber () + k] = cMatrixB.m_pTMatrix [i][k];
		}

	}


	return cMatrix;
}

