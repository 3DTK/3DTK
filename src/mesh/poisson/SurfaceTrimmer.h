/*
Added by: Xia Sun @ Zhejiang University
Based on: http://www.cs.jhu.edu/~misha/Code/PoissonRecon/Version5/
*/

/*
Copyright (c) 2013, Michael Kazhdan
All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list of
conditions and the following disclaimer. Redistributions in binary form must reproduce
the above copyright notice, this list of conditions and the following disclaimer
in the documentation and/or other materials provided with the distribution. 

Neither the name of the Johns Hopkins University nor the names of its contributors
may be used to endorse or promote products derived from this software without specific
prior written permission. 

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO THE IMPLIED WARRANTIES 
OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
TO, PROCUREMENT OF SUBSTITUTE  GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
DAMAGE.
*/

#include <stdio.h>
#include <stdlib.h>
#include <float.h>
#include <omp.h>
#include <vector>
#include "CmdLineParser.h"
#include "Geometry.h"
#include "Ply.h"
#include "MAT.h"
#include "Time.h"

#define FOR_RELEASE 1

long long EdgeKey( int key1 , int key2 )
{
	if( key1<key2 ) return ( ( (long long)key1 )<<32 ) | ( (long long)key2 );
	else            return ( ( (long long)key2 )<<32 ) | ( (long long)key1 );
}
template< class Real >
PlyValueVertex< Real > InterpolateVertices( const PlyValueVertex< Real >& v1 , const PlyValueVertex< Real >& v2 , const float& value )
{
	if( v1.value==v2.value ) return (v1+v2)/Real(2.);
	PlyValueVertex< Real > v;

	Real dx = (v1.value-value)/(v1.value-v2.value);
	for( int i=0 ; i<3 ; i++ ) v.point.coords[i]=v1.point.coords[i]*(1.f-dx)+v2.point.coords[i]*dx;
	v.value=v1.value*(1.f-dx)+v2.value*dx;
	return v;
}

template< class Real >
void ColorVertices( const std::vector< PlyValueVertex< Real > >& inVertices , std::vector< PlyColorVertex< Real > >& outVertices , float min , float max )
{
	outVertices.resize( inVertices.size() );
	for( size_t i=0 ; i<inVertices.size() ; i++ )
	{
		outVertices[i].point = inVertices[i].point;
		float temp = ( inVertices[i].value-min ) / ( max - min );
		temp = std::max< float >( 0.f , std::min< float >( 1.f , temp ) );
		temp *= 255;
		outVertices[i].color[0] = outVertices[i].color[1] = outVertices[i].color[2] = (int)temp;
	}
}
template< class Real >
void SmoothValues( std::vector< PlyValueVertex< Real > >& vertices , const std::vector< std::vector< int > >& polygons )
{
	std::vector< int > count( vertices.size() );
	std::vector< Real > sums( vertices.size() , 0 );
	for( size_t i=0 ; i<polygons.size() ; i++ )
	{
		int sz = int(polygons[i].size());
		for( int j=0 ; j<sz ; j++ )
		{
			int j1 = j , j2 = (j+1)%sz;
			int v1 = polygons[i][j1] , v2 = polygons[i][j2];
			count[v1]++ , count[v2]++;
			sums[v1] += vertices[v2].value , sums[v2] += vertices[v1].value;
		}
	}
	for( size_t i=0 ; i<vertices.size() ; i++ ) vertices[i].value = ( sums[i] + vertices[i].value ) / ( count[i] + 1 );
}
template< class Real >
void SmoothValues( std::vector< PlyValueVertex< Real > >& vertices , const std::vector< std::vector< int > >& polygons , Real min , Real max )
{
	std::vector< int > count( vertices.size() );
	std::vector< Real > sums( vertices.size() , 0 );
	for( int i=0 ; i<polygons.size() ; i++ ) for( int j=0 ; j<polygons[i].size() ; j++ )
	{
		int j1 = j , j2 = (j+1)%polygons[i].size();
		int v1 = polygons[i][j1] , v2 = polygons[i][j2];
		if( vertices[v1].value>min && vertices[v1].value<max ) count[v1]++ , sums[v1] += vertices[v2].value;
		if( vertices[v2].value>min && vertices[v2].value<max ) count[v2]++ , sums[v2] += vertices[v1].value;
	}
	for( int i=0 ; i<vertices.size() ; i++ ) vertices[i].value = ( sums[i] + vertices[i].value ) / ( count[i] + 1 );
}

template< class Real >
void SplitPolygon
	(
	const std::vector< int >& polygon ,
	std::vector< PlyValueVertex< Real > >& vertices , 
	std::vector< std::vector< int > >* ltPolygons , std::vector< std::vector< int > >* gtPolygons ,
	std::vector< bool >* ltFlags , std::vector< bool >* gtFlags ,
	hash_map< long long , int >& vertexTable ,
	Real trimValue
	)
{
	int sz = int( polygon.size() );
	std::vector< bool > gt( sz );
	int gtCount = 0;
	for( int j=0 ; j<sz ; j++ )
	{
		gt[j] = ( vertices[ polygon[j] ].value>trimValue );
		if( gt[j] ) gtCount++;
	}
	if     ( gtCount==sz ){ if( gtPolygons ) gtPolygons->push_back( polygon ) ; if( gtFlags ) gtFlags->push_back( false ); }
	else if( gtCount==0  ){ if( ltPolygons ) ltPolygons->push_back( polygon ) ; if( ltFlags ) ltFlags->push_back( false ); }
	else
	{
		int start;
		for( start=0 ; start<sz ; start++ ) if( gt[start] && !gt[(start+sz-1)%sz] ) break;

		bool gtFlag = true;
		std::vector< int > poly;

		// Add the initial vertex
		{
			int j1 = (start+int(sz)-1)%sz , j2 = start;
			int v1 = polygon[j1] , v2 = polygon[j2];
			int vIdx;
			hash_map< long long , int >::iterator iter = vertexTable.find( EdgeKey( v1 , v2 ) );
			if( iter==vertexTable.end() )
			{
				vertexTable[ EdgeKey( v1 , v2 ) ] = vIdx = int( vertices.size() );
				vertices.push_back( InterpolateVertices( vertices[v1] , vertices[v2] , trimValue ) );
			}
			else vIdx = iter->second;
			poly.push_back( vIdx );
		}

		for( int _j=0  ; _j<=sz ; _j++ )
		{
			int j1 = (_j+start+sz-1)%sz , j2 = (_j+start)%sz;
			int v1 = polygon[j1] , v2 = polygon[j2];
			if( gt[j2]==gtFlag ) poly.push_back( v2 );
			else
			{
				int vIdx;
				hash_map< long long , int >::iterator iter = vertexTable.find( EdgeKey( v1 , v2 ) );
				if( iter==vertexTable.end() )
				{
					vertexTable[ EdgeKey( v1 , v2 ) ] = vIdx = int( vertices.size() );
					vertices.push_back( InterpolateVertices( vertices[v1] , vertices[v2] , trimValue ) );
				}
				else vIdx = iter->second;
				poly.push_back( vIdx );
				if( gtFlag ){ if( gtPolygons ) gtPolygons->push_back( poly ) ; if( ltFlags ) ltFlags->push_back( true ); }
				else        { if( ltPolygons ) ltPolygons->push_back( poly ) ; if( gtFlags ) gtFlags->push_back( true ); }
				poly.clear() , poly.push_back( vIdx ) , poly.push_back( v2 );
				gtFlag = !gtFlag;
			}
		}
	}
}
template< class Real >
void Triangulate( const std::vector< PlyValueVertex< Real > >& vertices , const std::vector< std::vector< int > >& polygons , std::vector< std::vector< int > >& triangles )
{
	triangles.clear();
	for( size_t i=0 ; i<polygons.size() ; i++ )
		if( polygons.size()>3 )
		{
			MinimalAreaTriangulation< Real > mat;
			std::vector< Point3D< Real > > _vertices( polygons[i].size() );
			std::vector< TriangleIndex > _triangles;
			for( int j=0 ; j<int( polygons[i].size() ) ; j++ ) _vertices[j] = vertices[ polygons[i][j] ].point;
			mat.GetTriangulation( _vertices , _triangles );

			// Add the triangles to the mesh
			size_t idx = triangles.size();
			triangles.resize( idx+_triangles.size() );
			for( int j=0 ; j<int(_triangles.size()) ; j++ )
			{
				triangles[idx+j].resize(3);
				for( int k=0 ; k<3 ; k++ ) triangles[idx+j][k] = polygons[i][ _triangles[j].idx[k] ];
			}
		}
		else if( polygons[i].size()==3 ) triangles.push_back( polygons[i] );
}
template< class Vertex >
void RemoveHangingVertices( std::vector< Vertex >& vertices , std::vector< std::vector< int > >& polygons )
{
	hash_map< int , int > vMap;
	std::vector< bool > vertexFlags( vertices.size() , false );
	for( size_t i=0 ; i<polygons.size() ; i++ ) for( size_t j=0 ; j<polygons[i].size() ; j++ ) vertexFlags[ polygons[i][j] ] = true;
	int vCount = 0;
	for( int i=0 ; i<int(vertices.size()) ; i++ ) if( vertexFlags[i] ) vMap[i] = vCount++;
	for( size_t i=0 ; i<polygons.size() ; i++ ) for( size_t j=0 ; j<polygons[i].size() ; j++ ) polygons[i][j] = vMap[ polygons[i][j] ];

	std::vector< Vertex > _vertices( vCount );
	for( int i=0 ; i<int(vertices.size()) ; i++ ) if( vertexFlags[i] ) _vertices[ vMap[i] ] = vertices[i];
	vertices = _vertices;
}
void SetConnectedComponents( const std::vector< std::vector< int > >& polygons , std::vector< std::vector< int > >& components )
{
	std::vector< int > polygonRoots( polygons.size() );
	for( size_t i=0 ; i<polygons.size() ; i++ ) polygonRoots[i] = int(i);
	hash_map< long long , int > edgeTable;
	for( size_t i=0 ; i<polygons.size() ; i++ )
	{
		int sz = int( polygons[i].size() );
		for( int j=0 ; j<sz ; j++ )
		{
			int j1 = j , j2 = (j+1)%sz;
			int v1 = polygons[i][j1] , v2 = polygons[i][j2];
			long long eKey = EdgeKey( v1 , v2 );
			hash_map< long long , int >::iterator iter = edgeTable.find( eKey );
			if( iter==edgeTable.end() ) edgeTable[ eKey ] = int(i);
			else
			{
				int p = iter->second;
				while( polygonRoots[p]!=p )
				{
					int temp = polygonRoots[p];
					polygonRoots[p] = int(i);
					p = temp;
				}
				polygonRoots[p] = int(i);
			}
		}
	}
	for( size_t i=0 ; i<polygonRoots.size() ; i++ )
	{
		int p = int(i);
		while( polygonRoots[p]!=p ) p = polygonRoots[p];
		int root = p;
		p = int(i);
		while( polygonRoots[p]!=p )
		{
			int temp = polygonRoots[p];
			polygonRoots[p] = root;
			p = temp;
		}
	}
	int cCount = 0;
	hash_map< int , int > vMap;
	for( int i= 0 ; i<int(polygonRoots.size()) ; i++ ) if( polygonRoots[i]==i ) vMap[i] = cCount++;
	components.resize( cCount );
	for( int i=0 ; i<int(polygonRoots.size()) ; i++ ) components[ vMap[ polygonRoots[i] ] ].push_back(i);
}
template< class Real >
inline Point3D< Real > CrossProduct( Point3D< Real > p1 , Point3D< Real > p2 ){ return Point3D< Real >( p1[1]*p2[2]-p1[2]*p2[1] , p1[2]*p2[0]-p1[0]*p2[2] , p1[0]*p1[1]-p1[1]*p2[0] ); }
template< class Real >
double TriangleArea( Point3D< Real > v1 , Point3D< Real > v2 , Point3D< Real > v3 )
{
	Point3D< Real > n = CrossProduct( v2-v1 , v3-v1 );
	return sqrt( n[0]*n[0] + n[1]*n[1] + n[2]*n[2] ) / 2.;
}
template< class Real >
double PolygonArea( const std::vector< PlyValueVertex< Real > >& vertices , const std::vector< int >& polygon )
{
	if( polygon.size()<3 ) return 0.;
	else if( polygon.size()==3 ) return TriangleArea( vertices[polygon[0]].point , vertices[polygon[1]].point , vertices[polygon[2]].point );
	else
	{
		Point3D< Real > center;
		for( size_t i=0 ; i<polygon.size() ; i++ ) center += vertices[ polygon[i] ].point;
		center /= Real( polygon.size() );
		double area = 0;
		for( size_t i=0 ; i<polygon.size() ; i++ ) area += TriangleArea( center , vertices[ polygon[i] ].point , vertices[ polygon[ (i+1)%polygon.size() ] ].point );
		return area;
	}
}

int CallSurfaceTrimmer(float trimVal, std::vector<float*> &vts, std::vector<int*> &faces, std::vector<float*> &tVts, std::vector<int*> &tFaces) {
	float smoothVal;
	float min , max;
	float islandAreaRatio = 0.001;
	bool polygonMesh = false;
	bool colorRange = false;

	// construct vertices and polygons data
	std::vector<PlyValueVertex<float> > vertices;
	std::vector<std::vector<int> > polygons;
	for (int i = 0; i < vts.size(); ++i) {
		PlyValueVertex<float> v;
		v.point.coords[0] = vts[i][0];
		v.point.coords[1] = vts[i][1];
		v.point.coords[2] = vts[i][2];
		v.value = vts[i][3];
		vertices.push_back(v);
	}
	for (int i = 0; i < faces.size(); ++i) {
		std::vector<int> f = {faces[i][0], faces[i][1], faces[i][2]};
		polygons.push_back(f);
	}

// int ft;
// char** comments;
// #if 0
// 	if( Trim.set ) for( int i=0 ; i<Smooth.value ; i++ ) SmoothValues( vertices , polygons , Trim.value-0.5f , Trim.value+0.5f );
// 	else           for( int i=0 ; i<Smooth.value ; i++ ) SmoothValues( vertices , polygons );
// #else
// 	for( int i=0 ; i<smoothVal ; i++ ) SmoothValues( vertices , polygons );
// #endif 
	min = max = vertices[0].value;
	for( size_t i=0 ; i<vertices.size() ; i++ ) min = std::min< float >( min , vertices[i].value ) , max = std::max< float >( max , vertices[i].value );
	printf( "Value Range: [%f,%f]\n" , min , max );


	if( true )
	{
		hash_map< long long , int > vertexTable;
		std::vector< std::vector< int > > ltPolygons , gtPolygons;
		std::vector< bool > ltFlags , gtFlags;

		double t=Time();
		for( size_t i=0 ; i<polygons.size() ; i++ ) SplitPolygon( polygons[i] , vertices , &ltPolygons , &gtPolygons , &ltFlags , &gtFlags , vertexTable , trimVal );
		if( islandAreaRatio>0 )
		{
			std::vector< std::vector< int > > _ltPolygons , _gtPolygons;
			std::vector< std::vector< int > > ltComponents , gtComponents;
			SetConnectedComponents( ltPolygons , ltComponents );
			SetConnectedComponents( gtPolygons , gtComponents );
			std::vector< double > ltAreas( ltComponents.size() , 0. ) , gtAreas( gtComponents.size() , 0. );
			std::vector< bool > ltComponentFlags( ltComponents.size() , false ) , gtComponentFlags( gtComponents.size() , false );
			double area = 0.;
			for( size_t i=0 ; i<ltComponents.size() ; i++ )
			{
				for( size_t j=0 ; j<ltComponents[i].size() ; j++ )
				{
					ltAreas[i] += PolygonArea( vertices , ltPolygons[ ltComponents[i][j] ] );
					ltComponentFlags[i] = ( ltComponentFlags[i] || ltFlags[ ltComponents[i][j] ] );
				}
				area += ltAreas[i];
			}
			for( size_t i=0 ; i<gtComponents.size() ; i++ )
			{
				for( size_t j=0 ; j<gtComponents[i].size() ; j++ )
				{
					gtAreas[i] += PolygonArea( vertices , gtPolygons[ gtComponents[i][j] ] );
					gtComponentFlags[i] = ( gtComponentFlags[i] || gtFlags[ gtComponents[i][j] ] );
				}
				area += gtAreas[i];
			}
			for( size_t i=0 ; i<ltComponents.size() ; i++ )
			{
				if( ltAreas[i]<area*islandAreaRatio && ltComponentFlags[i] ) for( size_t j=0 ; j<ltComponents[i].size() ; j++ ) _gtPolygons.push_back( ltPolygons[ ltComponents[i][j] ] );
				else                                                               for( size_t j=0 ; j<ltComponents[i].size() ; j++ ) _ltPolygons.push_back( ltPolygons[ ltComponents[i][j] ] );
			}
			for( size_t i=0 ; i<gtComponents.size() ; i++ )
			{
				if( gtAreas[i]<area*islandAreaRatio && gtComponentFlags[i] ) for( size_t j=0 ; j<gtComponents[i].size() ; j++ ) _ltPolygons.push_back( gtPolygons[ gtComponents[i][j] ] );
				else                                                               for( size_t j=0 ; j<gtComponents[i].size() ; j++ ) _gtPolygons.push_back( gtPolygons[ gtComponents[i][j] ] );
			}
			ltPolygons = _ltPolygons , gtPolygons = _gtPolygons;
		}
		if( !polygonMesh )
		{
			{
				std::vector< std::vector< int > > polys = ltPolygons;
				Triangulate( vertices , ltPolygons , polys ) , ltPolygons = polys;
			}
			{
				std::vector< std::vector< int > > polys = gtPolygons;
				Triangulate( vertices , gtPolygons , polys ) , gtPolygons = polys;
			}
		}

		RemoveHangingVertices( vertices , gtPolygons );
		// sprintf( comments[commentNum++] , "#Trimmed In: %9.1f (s)" , Time()-t );
		// if( true ) PlyWritePolygons( "asdf" , vertices , gtPolygons , PlyValueVertex< float >::Properties , PlyValueVertex< float >::Components , ft , comments , 1 );
		for (int i = 0; i < vertices.size(); ++i) {
			float *v = new float[4];
			v[0] = vertices[i].point.coords[0]; 
			v[1] = vertices[i].point.coords[1]; 
			v[2] = vertices[i].point.coords[2];
			v[3] = vertices[i].value;
			tVts.push_back(v);
		}
		for (int i = 0; i < gtPolygons.size(); ++i) {
			int *f = new int[3];
			f[0] = gtPolygons[i][0]; f[1] = gtPolygons[i][1]; f[2] = gtPolygons[i][2];
			tFaces.push_back(f);
		}
	}
	else
	{
		// if( ColorRange.set ) min = ColorRange.values[0] , max = ColorRange.values[1];
		std::vector< PlyColorVertex< float > > outVertices;
		ColorVertices( vertices , outVertices , min , max );
		// if( Out.set ) PlyWritePolygons( Out.value , outVertices , polygons , PlyColorVertex< float >::Properties , PlyColorVertex< float >::Components , ft , comments , commentNum );
		for (int i = 0; i < outVertices.size(); ++i) {
			float *v = new float[3];
			v[0] = outVertices[i].point.coords[0]; 
			v[1] = outVertices[i].point.coords[1]; 
			v[2] = outVertices[i].point.coords[2];
			tVts.push_back(v);
		}
		for (int i = 0; i < polygons.size(); ++i) {
			int *f = new int[3];
			f[0] = polygons[i][0]; f[1] = polygons[i][1]; f[2] = polygons[i][2];
			tFaces.push_back(f);
		}
	}

	return 1;
}
