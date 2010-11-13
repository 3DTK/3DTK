/*
(***********************************************************************)
(*                                                                     *)
(* Wykobi Computational Geometry Library                               *)
(* Release Version 0.0.2                                               *)
(* http://www.wykobi.com                                               *)
(* Copyright (c) 2005-2007 Arash Partow, All Rights Reserved.          *)
(*                                                                     *)
(* The Wykobi computational geometry library and its components are    *)
(* supplied under the terms of the General Wykobi License agreement.   *)
(* The contents of the Wykobi computational geometry library and its   *)
(* components may not be copied or disclosed except in accordance with *)
(* the terms of that agreement.                                        *)
(*                                                                     *)
(* URL: http://www.wykobi.com/license.html                             *)
(*                                                                     *)
(***********************************************************************)
*/


#include "wykobi.hpp"
#include "wykobi_matrix.hpp"

namespace wykobi
{

   template<typename T, std::size_t M, std::size_t N>
   inline matrix<T,M,N>::matrix(const matrix<T,M,N>& m)
   {
      for(std::size_t x = 0; x < M; ++x)
      {
         for(std::size_t y = 0; y < N; ++y)
         {
            data[x][y] = m(x,y);
         }
      }
   }

   template<typename T, std::size_t M, std::size_t N>
   inline matrix<T,M,N>& matrix<T,M,N>::operator=(const matrix<T,M,N>& m)
   {
      if (this == &m)
        return *this;

      for(std::size_t x = 0; x < M; ++x)
      {
         for(std::size_t y = 0; y < N; ++y)
         {
            data[x][y] = m(x,y);
         }
      }
      return *this;
   }

   template<typename T, std::size_t M, std::size_t N>
   inline matrix<T,M,N>& matrix<T,M,N>::operator+=(const T& value)
   {
      for(std::size_t x = 0; x < M; ++x)
      {
         for(std::size_t y = 0; y < N; ++y)
         {
            data[x][y] += value;
         }
      }
      return (*this);
   }

   template<typename T, std::size_t M, std::size_t N>
   inline matrix<T,M,N>& matrix<T,M,N>::operator-=(const T& value)
   {
      for(std::size_t x = 0; x < M; ++x)
      {
         for(std::size_t y = 0; y < N; ++y)
         {
            data[x][y] -= value;
         }
      }
      return (*this);
   }

   template<typename T, std::size_t M, std::size_t N>
   inline matrix<T,M,N>& matrix<T,M,N>::operator*=(const T& value)
   {
      for(std::size_t x = 0; x < M; ++x)
      {
         for(std::size_t y = 0; y < N; ++y)
         {
            data[x][y] *= value;
         }
      }
      return (*this);
   }

   template<typename T, std::size_t M, std::size_t N>
   inline matrix<T,M,N>& matrix<T,M,N>::operator/=(const T& value)
   {
      for(std::size_t x = 0; x < M; ++x)
      {
         for(std::size_t y = 0; y < N; ++y)
         {
            data[x][y] /= value;
         }
      }
      return (*this);
   }

   template<typename T, std::size_t M, std::size_t N>
   inline matrix<T,M,N>& matrix<T,M,N>::operator+=(const matrix<T,M,N>& _matrix)
   {
      for(std::size_t x = 0; x < M; ++x)
      {
         for(std::size_t y = 0; y < N; ++y)
         {
            data[x][y] += _matrix(x,y);
         }
      }
      return (*this);
   }

   template<typename T, std::size_t M, std::size_t N>
   inline matrix<T,M,N>& matrix<T,M,N>::operator-=(const matrix<T,M,N>& _matrix)
   {
      for(std::size_t x = 0; x < M; ++x)
      {
         for(std::size_t y = 0; y < N; ++y)
         {
            data[x][y] -= _matrix(x,y);
         }
      }
      return (*this);
   }

   template<typename T, std::size_t M, std::size_t N>
   inline void matrix<T,M,N>::zero()
   {
      for(std::size_t x = 0; x < M; ++x)
      {
         for(std::size_t y = 0; y < N; ++y)
         {
            data[x][y] = T(0.0);
         }
      }
   }

   template<typename T, std::size_t M, std::size_t N>
   inline void matrix<T,M,N>::identity()
   {
      for(std::size_t x = 0; x < M; ++x)
      {
         for(std::size_t y = 0; y < N; ++y)
         {
            data[x][y] = ((x == y) ? T(1.0) : T(0.0));
         }
      }
   }

   template<typename T, std::size_t M, std::size_t N>
   inline void matrix<T,M,N>::swap(const unsigned int& x1,const unsigned int& y1,
                                   const unsigned int& x2,const unsigned int& y2)
   {
      T temp  = data[x1][y1];
      data[x1][y1] = data[x2][y2];
      data[x2][y2] = temp;
   }


   template<typename T>
   inline void transpose(matrix<T,1,1>& matrix)
   {
   }

   template<typename T>
   inline void transpose(matrix<T,2,2>& matrix)
   {
      matrix.swap(0,1,1,0);
   }

   template<typename T> inline void transpose(matrix<T,3,3>& matrix)
   {
      matrix.swap(0,1,1,0);
      matrix.swap(0,2,2,0);
      matrix.swap(1,2,2,1);
   }

   template<typename T> inline void transpose(matrix<T,4,4>& matrix)
   {
      matrix.swap(0,1,1,0);
      matrix.swap(0,2,2,0);
      matrix.swap(0,3,3,0);
      matrix.swap(1,2,2,1);
      matrix.swap(1,3,3,1);
      matrix.swap(2,3,3,2);
   }

   template<typename T>
   inline T det(const matrix<T,1,1>& matrix)
   {
      return matrix(0,0);
   }

   template<typename T>
   inline T det(const matrix<T,2,2>& matrix)
   {
      return matrix(0,0) * matrix(1,1) - matrix(1,0) * matrix(0,1);
   }

   template<typename T>
   inline T det(const matrix<T,3,3>& matrix)
   {
      return (matrix(0,0) * (matrix(1,1) * matrix(2,2) - matrix(1,2) * matrix(2,1)) -
              matrix(1,0) * (matrix(0,1) * matrix(2,2) - matrix(0,2) * matrix(2,1)) +
              matrix(2,0) * (matrix(0,1) * matrix(1,2) - matrix(0,2) * matrix(1,1)));
   }

   template<typename T>
   inline T det(const matrix<T,4,4>& matrix)
   {
      return matrix(3,0) * matrix(2,1) * matrix(1,2) * matrix(0,3) -
             matrix(2,0) * matrix(3,1) * matrix(1,2) * matrix(0,3) -
             matrix(3,0) * matrix(1,1) * matrix(2,2) * matrix(0,3) +
             matrix(1,0) * matrix(3,1) * matrix(2,2) * matrix(0,3) +
             matrix(2,0) * matrix(1,1) * matrix(3,2) * matrix(0,3) -
             matrix(1,0) * matrix(2,1) * matrix(3,2) * matrix(0,3) -
             matrix(3,0) * matrix(2,1) * matrix(0,2) * matrix(1,3) +
             matrix(2,0) * matrix(3,1) * matrix(0,2) * matrix(1,3) +
             matrix(3,0) * matrix(0,1) * matrix(2,2) * matrix(1,3) -
             matrix(0,0) * matrix(3,1) * matrix(2,2) * matrix(1,3) -
             matrix(2,0) * matrix(0,1) * matrix(3,2) * matrix(1,3) +
             matrix(0,0) * matrix(2,1) * matrix(3,2) * matrix(1,3) +
             matrix(3,0) * matrix(1,1) * matrix(0,2) * matrix(2,3) -
             matrix(1,0) * matrix(3,1) * matrix(0,2) * matrix(2,3) -
             matrix(3,0) * matrix(0,1) * matrix(1,2) * matrix(2,3) +
             matrix(0,0) * matrix(3,1) * matrix(1,2) * matrix(2,3) +
             matrix(1,0) * matrix(0,1) * matrix(3,2) * matrix(2,3) -
             matrix(0,0) * matrix(1,1) * matrix(3,2) * matrix(2,3) -
             matrix(2,0) * matrix(1,1) * matrix(0,2) * matrix(3,3) +
             matrix(1,0) * matrix(2,1) * matrix(0,2) * matrix(3,3) +
             matrix(2,0) * matrix(0,1) * matrix(1,2) * matrix(3,3) -
             matrix(0,0) * matrix(2,1) * matrix(1,2) * matrix(3,3) -
             matrix(1,0) * matrix(0,1) * matrix(2,2) * matrix(3,3) +
             matrix(0,0) * matrix(1,1) * matrix(2,2) * matrix(3,3);
   }

   template<typename T>
   inline matrix<T,2,2> inverse(const matrix<T,2,2>& m)
   {
      T det_value = det(m);
      matrix<T,2,2> _m;
      if (det_value != T(0.0))
      {
         _m = m;
         _m.swap(0,0,1,1);
         _m(1,0) *= T(-1.0);
         _m(0,1) *= T(-1.0);
         _m      *= (T(1.0) / det_value);
         return  _m;
      }
      else
         return _m;
   }

   template<typename T>
   inline matrix<T,3,3> inverse(const matrix<T,3,3>& m)
   {
      T det_value = det(m);
      matrix<T,3,3> _m;
      if (det_value != T(0.0))
      {
         _m(0,0) = m(1,1) * m(2,2) - m(1,2) * m(2,1);
         _m(1,0) = m(0,2) * m(2,1) - m(0,1) * m(2,2);
         _m(2,0) = m(0,1) * m(1,2) - m(0,2) * m(1,1);
         _m(0,1) = m(1,2) * m(2,0) - m(1,0) * m(2,2);
         _m(1,1) = m(0,0) * m(2,2) - m(0,2) * m(2,0);
         _m(2,1) = m(0,2) * m(1,0) - m(0,0) * m(1,2);
         _m(0,2) = m(1,0) * m(2,1) - m(1,1) * m(2,0);
         _m(1,2) = m(0,1) * m(2,0) - m(0,0) * m(2,1);
         _m(2,2) = m(0,0) * m(1,1) - m(0,1) * m(1,0);
         _m      *= (T(1.0) / det_value);
         return  _m;
      }
      else
         return _m;
   }

   template<typename T, std::size_t N>
   inline void inverse(matrix<T,N,N>& out_matrix, const matrix<T,N,N> in_matrix)
   {
      out_matrix = inverse(in_matrix);
   }

   template<typename T>
   inline void eigenvalues(const matrix<T,2,2>& matrix, T& eigenvalue1, T& eigenvalue2)
   {
      T delta = sqrt<T>(sqr<T>(matrix(0,0) - matrix(1,1)) + T(4.0) * matrix(1,0) * matrix(0,1));
      eigenvalue1 = T(0.5) * (matrix(0,0) + matrix(1,1) + delta);
      eigenvalue2 = T(0.5) * (matrix(0,0) + matrix(1,1) - delta);
   }

   template<typename T>
   inline void eigenvector(const matrix<T,2,2>& matrix,
                                 vector2d<T>& eigenvector1,
                                 vector2d<T>& eigenvector2)
   {
      T eigenvalue1;
      T eigenvalue2;
      eigenvalues(matrix,eigenvalue1,eigenvalue2);
      eigenvector1 = normalize(make_vector(T(-1.0) * matrix(1,0), matrix(0,0) - eigenvalue1));
      eigenvector2 = normalize(make_vector(T(-1.0) * matrix(1,0), matrix(0,0) - eigenvalue2));
   }

} // namespace wykobi
