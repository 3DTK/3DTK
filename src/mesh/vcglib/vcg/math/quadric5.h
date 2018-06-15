/****************************************************************************
* VCGLib                                                            o o     *
* Visual and Computer Graphics Library                            o     o   *
*                                                                _   O  _   *
* Copyright(C) 2004-2016                                           \/)\/    *
* Visual Computing Lab                                            /\/|      *
* ISTI - Italian National Research Council                           |      *
*                                                                    \      *
* All rights reserved.                                                      *
*                                                                           *
* This program is free software; you can redistribute it and/or modify      *   
* it under the terms of the GNU General Public License as published by      *
* the Free Software Foundation; either version 2 of the License, or         *
* (at your option) any later version.                                       *
*                                                                           *
* This program is distributed in the hope that it will be useful,           *
* but WITHOUT ANY WARRANTY; without even the implied warranty of            *
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the             *
* GNU General Public License (http://www.gnu.org/licenses/gpl.txt)          *
* for more details.                                                         *
*                                                                           *
****************************************************************************/

#ifndef __VCGLIB_QUADRIC5
#define __VCGLIB_QUADRIC5

#include <vcg/math/quadric.h>

namespace vcg
{
namespace math {

  typedef double ScalarType;

  // r = a-b
  void inline sub_vec5(const ScalarType a[5], const ScalarType b[5], ScalarType r[5])
  {
    r[0] = a[0] - b[0];
    r[1] = a[1] - b[1];
    r[2] = a[2] - b[2];
    r[3] = a[3] - b[3];
    r[4] = a[4] - b[4];
  }

  // returns the in-product a*b
  ScalarType inline inproduct5(const ScalarType a[5], const ScalarType b[5])
  {
    return a[0]*b[0]+a[1]*b[1]+a[2]*b[2]+a[3]*b[3]+a[4]*b[4];
  }

  // r = out-product of a*b
  void inline outproduct5(const ScalarType a[5], const ScalarType b[5], ScalarType r[5][5])
  {
    for(int i = 0; i < 5; i++)
      for(int j = 0; j < 5; j++)
        r[i][j] = a[i]*b[j];
  }

  // r = m*v
  void inline prod_matvec5(const ScalarType m[5][5], const ScalarType v[5], ScalarType r[5])
  {
    r[0] = inproduct5(m[0],v);
    r[1] = inproduct5(m[1],v);
    r[2] = inproduct5(m[2],v);
    r[3] = inproduct5(m[3],v);
    r[4] = inproduct5(m[4],v);
  }

  // r = (v transposed)*m
  void inline prod_vecmat5(ScalarType v[5],ScalarType m[5][5], ScalarType r[5])
  {
    for(int i = 0; i < 5; i++)
      for(int j = 0; j < 5; j++)
        r[j] = v[j]*m[j][i];
  }

  void inline normalize_vec5(ScalarType v[5])
  {
    ScalarType norma = sqrt(v[0]*v[0]+v[1]*v[1]+v[2]*v[2]+v[3]*v[3]+v[4]*v[4]);

    v[0]/=norma;
    v[1]/=norma;
    v[2]/=norma;
    v[3]/=norma;
    v[4]/=norma;
  }

  void inline normalize_vec3(ScalarType v[3])
  {
    ScalarType norma = sqrt(v[0]*v[0]+v[1]*v[1]+v[2]*v[2]);

    v[0]/=norma;
    v[1]/=norma;
    v[2]/=norma;

  }

  // dest -= m

  void inline sub_mat5(ScalarType dest[5][5],ScalarType m[5][5])
  {
    for(int i = 0; i < 5; i++)
      for(int j = 0; j < 5; j++)
        dest[i][j] -= m[i][j];
  }

  /* computes the symmetric matrix v*v */
  void inline symprod_vvt5(ScalarType dest[15],ScalarType v[5])
  {
    dest[0] = v[0]*v[0];
    dest[1] = v[0]*v[1];
    dest[2] = v[0]*v[2];
    dest[3] = v[0]*v[3];
    dest[4] = v[0]*v[4];
    dest[5] = v[1]*v[1];
    dest[6] = v[1]*v[2];
    dest[7] = v[1]*v[3];
    dest[8] = v[1]*v[4];
    dest[9] = v[2]*v[2];
    dest[10] = v[2]*v[3];
    dest[11] = v[2]*v[4];
    dest[12] = v[3]*v[3];
    dest[13] = v[3]*v[4];
    dest[14] = v[4]*v[4];

  }

  /* subtracts symmetric matrix */
  void inline sub_symmat5(ScalarType dest[15],ScalarType m[15])
  {
    for(int i = 0; i < 15; i++)
      dest[i] -= m[i];
  }

}
template<typename  Scalar>
class Quadric5
{
public:
    typedef Scalar ScalarType;
//	typedef  CMeshO::VertexType::FaceType FaceType;

    // the real quadric
    ScalarType a[15];
    ScalarType b[5];
    ScalarType c;

    inline Quadric5() { c = -1;}

    // Necessari se si utilizza stl microsoft
    // inline bool operator <  ( const Quadric & q ) const { return false; }
    // inline bool operator == ( const Quadric & q ) const { return true; }

    bool IsValid() const { return (c>=0); }
    void SetInvalid() { c = -1.0; }

    void Zero()																// Azzera le quadriche
    {
        a[0] = 0;
        a[1] = 0;
        a[2] = 0;
        a[3] = 0;
        a[4] = 0;
        a[5] = 0;
        a[6] = 0;
        a[7] = 0;
        a[8] = 0;
        a[9] = 0;
        a[10] = 0;
        a[11] = 0;
        a[12] = 0;
        a[13] = 0;
        a[14] = 0;

        b[0] = 0;
        b[1] = 0;
        b[2] = 0;
        b[3] = 0;
        b[4] = 0;

        c    = 0;
    }

    void swapv(ScalarType *vv, ScalarType *ww)
    {
        ScalarType tmp;
        for(int i = 0; i < 5; i++)
        {
            tmp = vv[i];
            vv[i] = ww[i];
            ww[i] = tmp;
        }
    }

    // Add the right subset of the current 5D quadric to a given 3D quadric.
  void AddtoQ3(math::Quadric<double> &q3) const
    {
        q3.a[0] += a[0];
        q3.a[1] += a[1];
        q3.a[2] += a[2];
        q3.a[3] += a[5];
        q3.a[4] += a[6];

        q3.a[5] += a[9];

        q3.b[0] += b[0];
        q3.b[1] += b[1];
        q3.b[2] += b[2];

        q3.c += c;

        assert(q3.IsValid());
    }


    // computes the real quadric and the geometric quadric using the face
    // The geometric quadric is added to the parameter qgeo
  template <class FaceType>
  void byFace(FaceType &f, math::Quadric<double> &q1, math::Quadric<double> &q2, math::Quadric<double> &q3, bool QualityQuadric, ScalarType BorderWeight)
    {
    typedef typename FaceType::VertexType::CoordType CoordType;
        double q = QualityFace(f);

        // if quality==0 then the geometrical quadric has just zeroes
        if(q)
        {
            byFace(f,true);			// computes the geometrical quadric
            AddtoQ3(q1);
            AddtoQ3(q2);
            AddtoQ3(q3);
            byFace(f,false);		// computes the real quadric
            for(int j=0;j<3;++j)
            {
                if( f.IsB(j) || QualityQuadric )
                {
                    Quadric5<double> temp;
                    TexCoord2f newtex;
                    CoordType newpoint = (f.P0(j)+f.P1(j))/2.0 + (f.N()/f.N().Norm())*Distance(f.P0(j),f.P1(j));
                    newtex.u() = (f.WT( (j+0)%3 ).u()+f.WT( (j+1)%3 ).u())/2.0;
                    newtex.v() = (f.WT( (j+0)%3 ).v()+f.WT( (j+1)%3 ).v())/2.0;
                    CoordType oldpoint = f.P2(j);
                    TexCoord2f oldtex = f.WT((j+2)%3);

                    f.P2(j)=newpoint;
                    f.WT((j+2)%3).u()=newtex.u();
                    f.WT((j+2)%3).v()=newtex.v();

                    temp.byFace(f,false);			// computes the full quadric
                    if(! f.IsB(j) ) temp.Scale(0.05);
          else temp.Scale(BorderWeight);
                    *this+=temp;

                    f.P2(j)=oldpoint;
                    f.WT((j+2)%3).u()=oldtex.u();
                    f.WT((j+2)%3).v()=oldtex.v();
                }
            }

        }
        else if(
            (f.WT(1).u()-f.WT(0).u()) * (f.WT(2).v()-f.WT(0).v()) -
            (f.WT(2).u()-f.WT(0).u()) * (f.WT(1).v()-f.WT(0).v())
            )
            byFace(f,false); // computes the real quadric
        else // the area is zero also in the texture space
        {
            a[0]=a[1]=a[2]=a[3]=a[4]=a[5]=a[6]=a[7]=a[8]=a[9]=a[10]=a[11]=a[12]=a[13]=a[14]=0;
            b[0]=b[1]=b[2]=b[3]=b[4]=0;
            c=0;
        }
    }


    // Computes the geometrical quadric if onlygeo == true and the real quadric if onlygeo == false
  template<class FaceType>
  void byFace(FaceType &fi, bool onlygeo)
    {
      //assert(onlygeo==false);
        ScalarType p[5];
        ScalarType q[5];
        ScalarType r[5];
//		ScalarType A[5][5];
        ScalarType e1[5];
        ScalarType e2[5];

        // computes p
        p[0] = fi.P(0).X();
        p[1] = fi.P(0).Y();
        p[2] = fi.P(0).Z();
        p[3] = fi.WT(0).u();
        p[4] = fi.WT(0).v();

        //  computes q
        q[0] = fi.P(1).X();
        q[1] = fi.P(1).Y();
        q[2] = fi.P(1).Z();
        q[3] = fi.WT(1).u();
        q[4] = fi.WT(1).v();

        //  computes r
        r[0] = fi.P(2).X();
        r[1] = fi.P(2).Y();
        r[2] = fi.P(2).Z();
        r[3] = fi.WT(2).u();
        r[4] = fi.WT(2).v();

        if(onlygeo)		{
            p[3] = 0; q[3] = 0;	r[3] = 0;
            p[4] = 0; q[4] = 0;	r[4] = 0;
        }

        ComputeE1E2(p,q,r,e1,e2);
        ComputeQuadricFromE1E2(e1,e2,p);

        if(IsValid())	return;
//		qDebug("Warning: failed to find a good 5D quadric try to permute stuff.");

        /*
        When c is very close to 0, loss of precision causes it to be computed as a negative number,
        which is invalid for a quadric. Vertex switches are performed in order to try to obtain a smaller
        loss of precision. The one with the smallest error is chosen.
        */
        double minerror = std::numeric_limits<double>::max();
        int minerror_index = 0;
        for(int i = 0; i < 7; i++) // tries the 6! configurations and chooses the one with the smallest error
        {
            switch(i)
            {
            case 0:
                break;
            case 1:
            case 3:
            case 5:
                swapv(q,r);
                break;
            case 2:
            case 4:
                swapv(p,r);
                break;
            case 6: // every swap has loss of precision
                swapv(p,r);
                for(int j = 0; j <= minerror_index; j++)
                {
                    switch(j)
                    {
                    case 0:
                        break;
                    case 1:
                    case 3:
                    case 5:
                        swapv(q,r);
                        break;
                    case 2:
                    case 4:
                        swapv(p,r);
                        break;
                    default:
                        assert(0);
                    }
                }
                minerror_index = -1;
                break;
            default:
                assert(0);
            }

      ComputeE1E2(p,q,r,e1,e2);
            ComputeQuadricFromE1E2(e1,e2,p);

            if(IsValid())
                return;
            else if (minerror_index == -1) // the one with the smallest error has been computed
                break;
            else if(-c < minerror)
            {
                minerror = -c;
                minerror_index = i;
            }
        }
        // failed to find a valid vertex switch

        // assert(-c <= 1e-8); // small error

        c = 0; // rounds up to zero
    }

// Given three 5D points it compute an orthonormal basis e1 e2
void ComputeE1E2 (const ScalarType p[5],	const	ScalarType q[5],	const	ScalarType r[5], ScalarType e1[5], ScalarType e2[5]) const
{
        ScalarType diffe[5];
        ScalarType tmpmat[5][5];
        ScalarType tmpvec[5];
//  computes e1
        math::sub_vec5(q,p,e1);
        math::normalize_vec5(e1);

        //  computes e2
        math::sub_vec5(r,p,diffe);
        math::outproduct5(e1,diffe,tmpmat);
        math::prod_matvec5(tmpmat,e1,tmpvec);
        math::sub_vec5(diffe,tmpvec,e2);
        math::normalize_vec5(e2);
}

// Given two orthonormal 5D vectors lying on the plane and one of the three points of the triangle compute the quadric.
// Note it uses the same notation of the original garland 98 paper.
void ComputeQuadricFromE1E2(ScalarType e1[5], ScalarType e2[5], ScalarType p[5] )
{
    // computes A
    a[0] = 1;
    a[1] = 0;
    a[2] = 0;
    a[3] = 0;
    a[4] = 0;
    a[5] = 1;
    a[6] = 0;
    a[7] = 0;
    a[8] = 0;
    a[9] = 1;
    a[10] = 0;
    a[11] = 0;
    a[12] = 1;
    a[13] = 0;
    a[14] = 1;

        ScalarType tmpsymmat[15];  // a compactly stored 5x5 symmetric matrix.
    math::symprod_vvt5(tmpsymmat,e1);
    math::sub_symmat5(a,tmpsymmat);
    math::symprod_vvt5(tmpsymmat,e2);
    math::sub_symmat5(a,tmpsymmat);

        ScalarType pe1;
        ScalarType pe2;

    pe1 = math::inproduct5(p,e1);
    pe2 = math::inproduct5(p,e2);

    //  computes b
        ScalarType tmpvec[5];

    tmpvec[0] = pe1*e1[0] + pe2*e2[0];
    tmpvec[1] = pe1*e1[1] + pe2*e2[1];
    tmpvec[2] = pe1*e1[2] + pe2*e2[2];
    tmpvec[3] = pe1*e1[3] + pe2*e2[3];
    tmpvec[4] = pe1*e1[4] + pe2*e2[4];

    math::sub_vec5(tmpvec,p,b);

    //  computes c
    c = math::inproduct5(p,p)-pe1*pe1-pe2*pe2;
}

  static bool Gauss55( ScalarType x[], ScalarType C[5][5+1] )
    {
        const ScalarType keps = (ScalarType)1e-6;
        int i,j,k;

        ScalarType eps;					// Determina valore cond.
            eps = math::Abs(C[0][0]);
        for(i=1;i<5;++i)
        {
            ScalarType t = math::Abs(C[i][i]);
            if( eps<t ) eps = t;
        }
        eps *= keps;

        for (i = 0; i < 5 - 1; ++i)    		// Ciclo di riduzione
        {
            int ma = i;				// Ricerca massimo pivot
            ScalarType vma = math::Abs( C[i][i] );
            for (k = i + 1; k < 5; k++)
            {
                ScalarType t = math::Abs( C[k][i] );
                if (t > vma)
                {
                    vma = t;
                    ma  = k;
                }
            }
            if (vma<eps)
                return false;        			// Matrice singolare
            if(i!=ma)				// Swap del massimo pivot
                for(k=0;k<=5;k++)
                {
                    ScalarType t = C[i][k];
                    C[i][k] = C[ma][k];
                    C[ma][k] = t;
                }

            for (k = i + 1; k < 5; k++)        	//  Riduzione
            {
                ScalarType s;
                s = C[k][i] / C[i][i];
                for (j = i+1; j <= 5; j++)
                    C[k][j] -= C[i][j] * s;
                C[k][i] = 0.0;
            }
        }

            // Controllo finale singolarita'
        if( math::Abs(C[5-1][5- 1])<eps)
            return false;

        for (i=5-1; i>=0; i--)			// Sostituzione
        {
            ScalarType t;
            for (t = 0.0, j = i + 1; j < 5; j++)
                t += C[i][j] * x[j];
            x[i] = (C[i][5] - t) / C[i][i];
      if(math::IsNAN(x[i])) return false;
      assert(!math::IsNAN(x[i]));
        }

        return true;
    }


    // computes the minimum of the quadric, imposing the geometrical constraint (geo[3] and geo[4] are obviosly ignored)
  bool MinimumWithGeoContraints(ScalarType x[5],const ScalarType geo[5]) const
    {
        x[0] = geo[0];
        x[1] = geo[1];
        x[2] = geo[2];

        ScalarType C3 = -(b[3]+geo[0]*a[3]+geo[1]*a[7]+geo[2]*a[10]);
        ScalarType C4 = -(b[4]+geo[0]*a[4]+geo[1]*a[8]+geo[2]*a[11]);

        if(a[12] != 0)
        {
            double tmp = (a[14]-a[13]*a[13]/a[12]);

            if(tmp == 0)
                return false;

            x[4] = (C4 - a[13]*C3/a[12])/ tmp;
            x[3] = (C3 - a[13]*x[4])/a[12];
        }
        else
        {
            if(a[13] == 0)
                return false;

            x[4] = C3/a[13];
            x[3] = (C4 - a[14]*x[4])/a[13];
        }
    for(int i=0;i<5;++i)
      if( math::IsNAN(x[i])) return false;
      //assert(!math::IsNAN(x[i]));

        return true;
    }

    // computes the minimum of the quadric
  bool Minimum(ScalarType x[5]) const
    {
            ScalarType C[5][6];

            C[0][0] = a[0];
            C[0][1] = a[1];
            C[0][2] = a[2];
            C[0][3] = a[3];
            C[0][4] = a[4];
            C[1][0] = a[1];
            C[1][1] = a[5];
            C[1][2] = a[6];
            C[1][3] = a[7];
            C[1][4] = a[8];
            C[2][0] = a[2];
            C[2][1] = a[6];
            C[2][2] = a[9];
            C[2][3] = a[10];
            C[2][4] = a[11];
            C[3][0] = a[3];
            C[3][1] = a[7];
            C[3][2] = a[10];
            C[3][3] = a[12];
            C[3][4] = a[13];
            C[4][0] = a[4];
            C[4][1] = a[8];
            C[4][2] = a[11];
            C[4][3] = a[13];
            C[4][4] = a[14];

            C[0][5]=-b[0];
            C[1][5]=-b[1];
            C[2][5]=-b[2];
            C[3][5]=-b[3];
            C[4][5]=-b[4];

            return Gauss55(&(x[0]),C);
    }

    void operator = ( const Quadric5<double> & q )			// Assegna una quadrica
    {
        //assert( IsValid() );
        assert( q.IsValid() );

        a[0] = q.a[0];
        a[1] = q.a[1];
        a[2] = q.a[2];
        a[3] = q.a[3];
        a[4] = q.a[4];
        a[5] = q.a[5];
        a[6] = q.a[6];
        a[7] = q.a[7];
        a[8] = q.a[8];
        a[9] = q.a[9];
        a[10] = q.a[10];
        a[11] = q.a[11];
        a[12] = q.a[12];
        a[13] = q.a[13];
        a[14] = q.a[14];

        b[0] = q.b[0];
        b[1] = q.b[1];
        b[2] = q.b[2];
        b[3] = q.b[3];
        b[4] = q.b[4];

        c    = q.c;
    }

    // sums the geometrical and the real quadrics
    void operator += ( const Quadric5<double> & q )
    {
        //assert( IsValid() );
        assert( q.IsValid() );

        a[0] += q.a[0];
        a[1] += q.a[1];
        a[2] += q.a[2];
        a[3] += q.a[3];
        a[4] += q.a[4];
        a[5] += q.a[5];
        a[6] += q.a[6];
        a[7] += q.a[7];
        a[8] += q.a[8];
        a[9] += q.a[9];
        a[10] += q.a[10];
        a[11] += q.a[11];
        a[12] += q.a[12];
        a[13] += q.a[13];
        a[14] += q.a[14];

        b[0] += q.b[0];
        b[1] += q.b[1];
        b[2] += q.b[2];
        b[3] += q.b[3];
        b[4] += q.b[4];

        c    += q.c;

    }

/*
it sums the real quadric of the class with a quadric obtained by the geometrical quadric of the vertex.
This quadric is obtained extending to five dimensions the geometrical quadric and simulating that it has been
obtained by sums of 5-dimension quadrics which were computed using vertexes and faces with always the same values
in the fourth and fifth dimensions (respectly the function parameter u and the function parameter v).
this allows to simulate the inexistant continuity in vertexes with multiple texture coords
however this continuity is still inexistant, so even if the algorithm makes a good collapse with this expedient,it obviously
computes bad the priority......this should be adjusted with the extra weight user parameter through.....

*/
    void inline Sum3 (const math::Quadric<double> & q3, float u, float v)
  {
        assert( q3.IsValid() );

        a[0] += q3.a[0];
        a[1] += q3.a[1];
        a[2] += q3.a[2];

        a[5] += q3.a[3];
        a[6] += q3.a[4];

        a[9] += q3.a[5];

        a[12] += 1;
        a[14] += 1;

        b[0] += q3.b[0];
        b[1] += q3.b[1];
        b[2] += q3.b[2];

        b[3] -= u;
        b[4] -= v;

        c    += q3.c + u*u + v*v;

    }

    void Scale(ScalarType val)
    {
     for(int i=0;i<15;++i)
         a[i]*=val;
     for(int i=0;i<5;++i)
         b[i]*=val;
     c*=val;
    }

  // returns the quadric value in v
    ScalarType Apply(const ScalarType v[5]) const
    {

        assert( IsValid() );

        ScalarType tmpmat[5][5];
        ScalarType tmpvec[5];

        tmpmat[0][0] = a[0];
        tmpmat[0][1] = tmpmat[1][0] = a[1];
        tmpmat[0][2] = tmpmat[2][0] = a[2];
        tmpmat[0][3] = tmpmat[3][0] = a[3];
        tmpmat[0][4] = tmpmat[4][0] = a[4];

        tmpmat[1][1] = a[5];
        tmpmat[1][2] = tmpmat[2][1] = a[6];
        tmpmat[1][3] = tmpmat[3][1] = a[7];
        tmpmat[1][4] = tmpmat[4][1] = a[8];

        tmpmat[2][2] = a[9];
        tmpmat[2][3] = tmpmat[3][2] = a[10];
        tmpmat[2][4] = tmpmat[4][2] = a[11];

        tmpmat[3][3] = a[12];
        tmpmat[3][4] = tmpmat[4][3] = a[13];

        tmpmat[4][4] = a[14];

        math::prod_matvec5(tmpmat,v,tmpvec);

        return  math::inproduct5(v,tmpvec) + 2*math::inproduct5(b,v) + c;

    }
};

} // end namespace vcg;
#endif
