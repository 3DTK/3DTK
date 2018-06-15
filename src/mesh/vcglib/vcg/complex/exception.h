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
#ifndef __VCG_EXCEPTION_H
#define __VCG_EXCEPTION_H

namespace vcg
{
class MissingComponentException : public std::runtime_error
{
public:
  MissingComponentException(const std::string &err):std::runtime_error(err)
  {
    std::cout << "Missing Component Exception -" << err << "- \n";
  }
    virtual const char *what() const throw ()
    {
      static char buf[128]="Missing Component";
      return buf;
    }
};

class MissingCompactnessException : public std::runtime_error
{
public:
  MissingCompactnessException(const std::string &err):std::runtime_error(err)
  {
    std::cout << "Lack of Compactness Exception -" << err << "- \n";
  }
    virtual const char *what() const throw ()
    {
      static char buf[128]="Lack of Compactness";
      return buf;
    }
};

class MissingTriangularRequirementException : public std::runtime_error
{
public:
  MissingTriangularRequirementException(const std::string &err):std::runtime_error(err)
  {
    std::cout << "Mesh has to be composed by triangle and not polygons -" << err << "- \n";
  }

    virtual const char *what() const throw ()
    {
      static char buf[128]="Mesh has to be composed by triangle and not polygons";
      return buf;
    }
};

class MissingPolygonalRequirementException : public std::runtime_error
{
public:
  MissingPolygonalRequirementException(const std::string &err):std::runtime_error(err)
  {
    std::cout << "Mesh has to be composed by polygonal faces (not plain triangles) -" << err << "- \n";
  }

    virtual const char *what() const throw ()
    {
      static char buf[128]="Mesh has to be composed by polygonal faces (not plain triangles) ";
      return buf;
    }
};

class MissingPreconditionException : public std::runtime_error
{
public:
  MissingPreconditionException(const std::string &err):std::runtime_error(err)
  {
    std::cout << "Mesh does not satisfy the following precondition:" << err << "- \n";
  }

    virtual const char *what() const throw ()
    {
      static char buf[128]="Mesh does not satisfy precondition";
      return buf;
    }
};

} // end namespace vcg
#endif // EXCEPTION_H
