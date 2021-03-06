#ifndef __SPHERICAL_QUADTREE_H__
#define __SPHERICAL_QUADTREE_H__

#include <array>
#include <boost/functional/hash.hpp>
#include <memory>
#include <unordered_map>
#include <vector>

namespace std
{
template <> struct hash<std::pair<size_t, size_t>> {
	size_t operator()(std::pair<size_t, size_t> const &arg) const noexcept
	{
		return boost::hash_value(arg);
	}
};
} // namespace std

#include "slam6d/data_types.h"

class QuadNode;

class QuadNode
{
      private:
	std::vector<std::array<double, 3>> const &pts;
	std::vector<size_t> indices;
	double ccp[3]; // circumcircle center
	double ccr;    // angle between circle center and edge
	double area;   // area of triangle on sphere surface
	bool isleaf;
	std::unique_ptr<QuadNode> t1;
	std::unique_ptr<QuadNode> t2;
	std::unique_ptr<QuadNode> t3;
	std::unique_ptr<QuadNode> t4;

	std::vector<size_t> getall() const;

      public:
	QuadNode(size_t, size_t, size_t, std::vector<size_t> const &,
		 std::vector<std::array<double, 3>> const &,
		 std::vector<std::array<double, 3>> &,
		 std::unordered_map<std::pair<size_t, size_t>, size_t> &);

	std::vector<size_t> search(double p[3], const double r) const;
	std::vector<size_t> reduce(double theta, double cap_area,
				   int numpts) const;
};

class QuadTree
{
      private:
	std::array<std::unique_ptr<QuadNode>, 8> trees;
	std::unordered_map<std::pair<size_t, size_t>, size_t> middlemap;
	std::vector<std::array<double, 3>> pts;
	std::vector<std::array<double, 3>> vertices;

      public:
	QuadTree(DataXYZ const &);

	std::vector<size_t> search(double p[3], const double r) const;
	std::vector<size_t> reduce(double red, int octree) const;
};

#endif
