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

#ifndef VCGADDONS_H
#define VCGADDONS_H


namespace vcg {

template <typename Scalar>
inline Scalar MaxCoeff(Point3<Scalar> const & p)
{
	return std::max(std::max(p.X(), p.Y()), p.Z());
}

//template <typename Scalar>
//inline Scalar MinCoeff(Point3<Scalar> const & p)
//{
//	return std::min(std::min(p.X(), p.Y()), p.Z());
//}

template <typename Scalar>
inline Scalar Dot(Point3<Scalar> const & p1, Point3<Scalar> const & p2)
{
	return p1.X() * p2.X() + p1.Y() * p2.Y() + p1.Z() * p2.Z();
}

//template <typename Scalar>
//inline Point3<Scalar> Cross(Point3<Scalar> const & p1, Point3<Scalar> const & p2)
//{
//	return p1 ^ p2;
//}

//template <typename Scalar>
//inline Point3<Scalar> CwiseAdd(Point3<Scalar> const & p1, Scalar s)
//{
//	return Point3<Scalar>(p1.X() + s, p1.Y() + s, p1.Z() + s);
//}

template <typename Scalar>
inline int MaxCoeffId(Point3<Scalar> const & p)
{
	if (p.X()>p.Y())
		return p.X()>p.Z() ? 0 : 2;
	else
		return p.Y()>p.Z() ? 1 : 2;
}

//template <typename Scalar>
//inline int MinCoeffId(Point3<Scalar> const & p)
//{
//	if (p.X()<p.Y())
//		return p.X()<p.Z() ? 0 : 2;
//	else
//		return p.Y()<p.Z() ? 1 : 2;
//}

//template <typename ToType, typename Scalar>
//inline Point3<ToType> Point3Cast(const Point3<Scalar>& p)
//{
//	return Point3<ToType>(p.X(), p.Y(), p.Z());
//}

//template<class Scalar>
//Scalar Distance(const Point3<Scalar> &p, const Box3<Scalar> &bbox)
//{
//	Scalar dist2 = 0.;
//	Scalar aux;
//	for (int k=0 ; k<3 ; ++k)
//	{
//		if ( (aux = (p[k]-bbox.min[k]))<0. )
//			dist2 += aux*aux;
//		else if ( (aux = (bbox.max[k]-p[k]))<0. )
//			dist2 += aux*aux;
//	}
//	return sqrt(dist2);
//}

}

#endif
