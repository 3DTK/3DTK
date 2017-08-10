#include <iostream>
#include <fstream>

#include "slam6d/globals.icc"
#include "slam6d/scan.h"
#include "slam6d/kdIndexed.h"


int main(int argc, char **argv)
{
	int start = 2, end = 2;
	Scan::openDirectory(false, "/home/3dtk/wolfsburg20151110.uos.merged/reduced/", XYZR, start, end);

	if (Scan::allScans.size() != 1) {
		std::cerr << "invalid number of scans" << std::endl;
		return 1;
	}

	Scan *scan = *(Scan::allScans.begin());
	DataXYZ xyz = scan->get("xyz");
	unsigned int nPoints = xyz.size();

	// at the beginning every scan point is in its own group
	std::vector<std::vector <unsigned int> > group2points;
	group2points.reserve(nPoints);
	for (unsigned int i = 0; i < nPoints; ++i) {
		group2points.push_back(std::vector<unsigned int>());
		group2points[i].push_back(i);
	}

	std::vector<unsigned int> point2group;
	point2group.reserve(nPoints);
	for (unsigned int i = 0; i < nPoints; ++i) {
		point2group.push_back(i);
	}

	// build a KDtree
	double** pa = new double*[nPoints];
	for (unsigned int i = 0; i < nPoints; ++i) {
		pa[i] = new double[3];
		pa[i][0] = xyz[i][0];
		pa[i][1] = xyz[i][1];
		pa[i][2] = xyz[i][2];
	}

	std::cerr << "building KDtree..." << std::endl;
	time_t before = time(NULL);
	KDtreeIndexed t(pa, nPoints);
	time_t after = time(NULL);
	std::cerr << "took: " << difftime(after, before) << " seconds" << std::endl;

	std::cerr << "segmenting..." << std::endl;
	before = time(NULL);
	// go through all points
	for(unsigned int i = 0; i < nPoints; ++i){
		std::cerr << (i*100.0)/nPoints << " %\r";
		std::cerr.flush();

		unsigned int current_group = point2group[i];

		std::vector<size_t> neighbours = t.fixedRangeSearch(pa[i], 20, /* thread_num = */ 0);

		// check if any of the current point's neighbors has already been
		// grouped before (its group index is smaller than i)
		unsigned int smallest_group = nPoints;
		for (const auto &it : neighbours) {
			if (point2group[it] < smallest_group) {
				smallest_group = point2group[it];
			}
		}

		// if none of the neighbors has been handled before, just add all
		// the neighbors to the current group
		if (smallest_group >= i) {
			for (const auto &it : neighbours) {
				group2points[current_group].push_back(it);
				group2points[point2group[it]].clear();
				point2group[it] = current_group;
			}
			continue;
		}

		// if we end up here, some of the neighbors have been handled
		// before
		// We now go through all groups of all the neighbors and make all
		// points in each of these groups part of the group with the
		// lowest index
		// this effectively merges groups
		for (const auto &it : neighbours) {
			current_group = point2group[it];

			// nothing to do for the group with the smallest index
			if (current_group == smallest_group)
				continue;

			group2points[smallest_group].insert(group2points[smallest_group].end(), group2points[current_group].begin(), group2points[current_group].end());
			for (const auto &it2 : group2points[current_group]) {
				point2group[it2] = smallest_group;
			}
			group2points[it].clear();
		}
	}
	std::cerr << std::endl;
	after = time(NULL);
	std::cerr << "took: " << difftime(after, before) << " seconds" << std::endl;

	std::cerr << "writing files..." << std::endl;
	unsigned int current_idx = 0;
	for (unsigned int i = 0; i < nPoints; ++i) {
		if (group2points[i].size() == 0)
			continue;
		std::ofstream myfile;
		myfile.open("/tmp/test/scan" + to_string(current_idx, 3) + ".3d");
		for (unsigned int j = 0; j < group2points[i].size(); ++j) {
			myfile << pa[group2points[i][j]][0] << " " << pa[group2points[i][j]][1] << " " << pa[group2points[i][j]][2] << std::endl;
		}
		myfile.close();
		std::ofstream mypose;
		mypose.open("/tmp/test/scan" + to_string(current_idx, 3) + ".pose");
		mypose << "0 0 0" << std::endl;
		mypose << "0 0 0" << std::endl;
		mypose.close();
		current_idx += 1;
	}

	for (unsigned int i = 0; i < nPoints; ++i) {
		delete[] pa[i];
	}
	delete[] pa;
}
