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
#ifndef COMPONENT_SPH_
#define COMPONENT_SPH_

#include <vcg/math/spherical_harmonics.h>

namespace vcg {
namespace vertex {

template <class A, class T> class Sph: public T
{
public:
	typedef A SphType;
	SphType &SH() { return _harmonics; }
	const SphType &cSH() const { return _harmonics; }
	template < class LeftV>
	//	void ImportData(const LeftV  & left ) { SH() = left.cSH(); T::ImportData( left); }
	void ImportData(const LeftV  & left ) { T::ImportData( left); }
	static bool HasSH()   { return true; }
	static void Name(std::vector<std::string> & name){name.push_back(std::string("Spherical Harmonics"));T::Name(name);}

private:
  SphType _harmonics;
};

template <class T> class Sph9f: public Sph<vcg::math::SphericalHarmonics<float, 3>, T> {
	public:	static void Name(std::vector<std::string> & name){name.push_back(std::string("Sph9f"));T::Name(name);}
};

template <class T> class Sph16f: public Sph<vcg::math::SphericalHarmonics<float, 4>, T> {
	public:	static void Name(std::vector<std::string> & name){name.push_back(std::string("Sph16f"));T::Name(name);}
};

template <class T> class Sph25f: public Sph<vcg::math::SphericalHarmonics<float, 5>, T> {
	public:	static void Name(std::vector<std::string> & name){name.push_back(std::string("Sph25f"));T::Name(name);}
};

template <class T> class Sph36f: public Sph<vcg::math::SphericalHarmonics<float, 6>, T> {
	public:	static void Name(std::vector<std::string> & name){name.push_back(std::string("Sph36f"));T::Name(name);}
};

template <class T> class Sph49f: public Sph<vcg::math::SphericalHarmonics<float, 7>, T> {
	public:	static void Name(std::vector<std::string> & name){name.push_back(std::string("Sph49f"));T::Name(name);}
};

template <class T> class Sph9d: public Sph<vcg::math::SphericalHarmonics<double, 3>, T> {
	public:	static void Name(std::vector<std::string> & name){name.push_back(std::string("Sph9d"));T::Name(name);}
};

template <class T> class Sph16d: public Sph<vcg::math::SphericalHarmonics<double, 4>, T> {
	public:	static void Name(std::vector<std::string> & name){name.push_back(std::string("Sph16d"));T::Name(name);}
};

template <class T> class Sph25d: public Sph<vcg::math::SphericalHarmonics<double, 5>, T> {
	public:	static void Name(std::vector<std::string> & name){name.push_back(std::string("Sph25d"));T::Name(name);}
};

template <class T> class Sph36d: public Sph<vcg::math::SphericalHarmonics<double, 6>, T> {
	public:	static void Name(std::vector<std::string> & name){name.push_back(std::string("Sph36d"));T::Name(name);}
};

}}

#endif /*COMPONENT_H_*/
