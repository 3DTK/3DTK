#include <peopleremover/common.h>

#ifdef WITH_MMAP_SCAN
#include <sys/mman.h>
#include <fcntl.h>
#include <cerrno>
#include <cstring>
#endif

namespace po = boost::program_options;

int main(int argc, char* argv[])
{
	ssize_t start, end;
	IOType format;
	double fuzz;
	double voxel_size;
	size_t diff, normal_knearest;
	size_t cluster_size;
	normal_method_t normal_method;
	maxrange_method_t maxrange_method;
	std::string maskdir;
	std::string staticdir;
	std::string dir;
	bool no_subvoxel_accuracy;
	bool write_maxranges;
	int jobs;
#ifdef WITH_MMAP_SCAN
	std::string cachedir;
#endif

	parse_cmdline(argc, argv, start, end, format, fuzz, voxel_size, diff,
			normal_knearest, cluster_size, normal_method, maxrange_method,
			maskdir, staticdir, dir, no_subvoxel_accuracy, write_maxranges,
			jobs
#ifdef WITH_MMAP_SCAN
			, cachedir
#endif
			);

	double voxel_diagonal = sqrt(3*voxel_size*voxel_size);

	std::cerr << "dir: " << dir << std::endl;

	double elapsed;
	struct timespec before, after;
	clock_gettime(CLOCK_MONOTONIC, &before);

#ifdef WITH_MMAP_SCAN
	Scan::openDirectory(false, dir, format, start, end, cachedir);
#else
	Scan::openDirectory(false, dir, format, start, end);
#endif
	if(Scan::allScans.size() == 0) {
		std::cerr << "No scans found. Did you use the correct format?" << std::endl;
		exit(-1);
	}

	std::unordered_map<size_t, DataXYZ> points_by_slice;
	std::unordered_map<size_t, DataReflectance> reflectances_by_slice;
	std::unordered_map<size_t, DataXYZ> orig_points_by_slice;
	std::unordered_map<size_t, std::tuple<const double *, const double *, const double *>> trajectory;
#ifdef WITH_MMAP_SCAN
	std::unordered_map<size_t, int> mmap_fds;
#endif
	std::cerr << "size: " << Scan::allScans.size() << std::endl;
	std::vector<size_t> scanorder;
	for (size_t id = 0; id < Scan::allScans.size(); ++id) {
		size_t i = id+start;
		scanorder.push_back(i);
		Scan *scan = Scan::allScans[id];
		/* The range filter must be set *before* transformAll() because
		 * otherwise, transformAll will move the point coordinates such that
		 * the rangeFilter doesn't filter the right points anymore. This in
		 * turn can lead to the situation that when retrieving the reflectance
		 * values, some reflectance values are not returned because when
		 * reading them in, those get filtered by the *original* point
		 * coordinates. This in turn can lead to the situation that the vector
		 * for xyz and reflectance are of different length.
		 */
		scan->setRangeFilter(-1, voxel_diagonal);
		DataXYZ xyz_orig(scan->get("xyz"));
		// copy points
		size_t raw_orig_data_size = xyz_orig.size() * sizeof(double) * 3;
		unsigned char* xyz_orig_data;
#ifdef WITH_MMAP_SCAN
		if (!cachedir.empty()) {
			char filename[] = "ppl_XXXXXX";
			int fd = mkstemp(filename);
			if (fd == -1) {
				throw std::runtime_error("cannot create temporary file");
			}
			// by unlinking the file now, we make sure that there are no leftover
			// files even if the process is killed
			unlink(filename);
			int ret = fallocate(fd, 0, 0, raw_orig_data_size);
			if (ret == -1) {
				throw std::runtime_error("cannot fallocate");
			}
			xyz_orig_data = (unsigned char *)mmap(NULL, raw_orig_data_size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
			if (xyz_orig_data == MAP_FAILED) {
				throw std::runtime_error(std::string("cannot mmap: ")+std::string(std::strerror(errno))+std::string(" ")+std::to_string(errno));
			}
			mmap_fds[i] = fd;
		} else {
#endif
			xyz_orig_data = new unsigned char[raw_orig_data_size];
#ifdef WITH_MMAP_SCAN
		}
#endif
		memcpy(xyz_orig_data, xyz_orig.get_raw_pointer(), raw_orig_data_size);
		orig_points_by_slice[i] = DataPointer(xyz_orig_data, raw_orig_data_size);
		// now that the original coordinates are saved, transform
		scan->transformAll(scan->get_transMatOrg());
		trajectory[i] = std::make_tuple(scan->get_rPos(), scan->get_rPosTheta(), scan->get_transMatOrg());
		DataXYZ xyz(scan->get("xyz"));
		DataReflectance refl(scan->get("reflectance"));
		if (refl.size() != 0) {
			if (xyz.size() != refl.size() || xyz_orig.size() != refl.size()) {
				std::cerr << "refl mismatch: " << xyz.size() << " vs. " << refl.size() << std::endl;
				exit(1);
			}
			reflectances_by_slice[i] = refl;
		}
		points_by_slice[i] = xyz;
		std::cerr << "number of points in scan " << i << ": " << xyz.size() << std::endl;
	}
	clock_gettime(CLOCK_MONOTONIC, &after);
	elapsed = (after.tv_sec - before.tv_sec);
	elapsed += (after.tv_nsec - before.tv_nsec) / 1000000000.0;
	std::cerr << "took: " << elapsed << " seconds" << std::endl;

	std::cerr << "calculate voxel occupation" << std::endl;

	clock_gettime(CLOCK_MONOTONIC, &before);
	std::unordered_map<struct voxel, std::set<size_t>> voxel_occupied_by_slice;
	std::cerr << "0 %\r";
	std::cerr.flush();
	size_t done = 0;
	for (std::pair<size_t, DataXYZ> element : points_by_slice) {
		for (size_t i = 0; i < element.second.size(); ++i) {
			voxel_occupied_by_slice[voxel_of_point(element.second[i],voxel_size)].insert(element.first);
		}
		std::cerr << ((done+1)*100.0f/points_by_slice.size()) << " %\r";
		std::cerr.flush();
		done += 1;
	}
	std::cerr << std::endl;
	clock_gettime(CLOCK_MONOTONIC, &after);
	elapsed = (after.tv_sec - before.tv_sec);
	elapsed += (after.tv_nsec - before.tv_nsec) / 1000000000.0;
	std::cerr << "took: " << elapsed << " seconds" << std::endl;

	if (voxel_occupied_by_slice.size() == 0) {
		std::cerr << "no voxel occupied" << std::endl;
		exit(1);
	}

	std::cerr << "occupied voxels: " << voxel_occupied_by_slice.size() << std::endl;

	if (maxrange_method != NONE) {
		std::cerr << "compute maxranges and walk voxels" << std::endl;
		clock_gettime(CLOCK_MONOTONIC, &before);
	} else {
		std::cerr << "walk voxels" << std::endl;
		clock_gettime(CLOCK_MONOTONIC, &before);
	}
	std::cerr << "0 %\r";
	std::cerr.flush();

	std::set<struct voxel> free_voxels;

	/*
	 *  we need a separate variable to keep track of how many scans were done
	 *  because they are not done in order when execution happens in parallel
	 */
	done = 0;
#ifdef _OPENMP
	omp_set_num_threads(jobs);
#pragma omp parallel for schedule(dynamic)
#endif
	for (size_t idx = 0; idx < scanorder.size(); ++idx) {
		size_t i = scanorder[idx];
		// FIXME: this is just wasting memory
		std::vector<double> maxranges(points_by_slice[i].size(), std::numeric_limits<double>::infinity());

		if (maxrange_method == NORMALS) {
			if (normal_method == KNEAREST_GLOBAL || normal_method == RANGE_GLOBAL) {
				exit(1);
			}
			compute_maxranges(maxranges, orig_points_by_slice[i], normal_method, voxel_diagonal, fuzz);
		} else if (maxrange_method == ONENEAREST) {
			exit(1);
		}
		// FIXME: fuzz also needs to be applied with maxrange_method == NONE

		if (write_maxranges) {
			std::cerr << "write maxranges" << std::endl;
			exit(1);
		}

		std::set<struct voxel> free;
		for (size_t j = 0; j < points_by_slice[i].size(); ++j) {
			double p[3] = {points_by_slice[i][j][0], points_by_slice[i][j][1], points_by_slice[i][j][2]};
			if (maxranges[j] != std::numeric_limits<double>::infinity()) {
				double maxrange = maxranges[j];
				double r = Len(orig_points_by_slice[i][j]);
				double factor = maxrange/r;
				p[0] = orig_points_by_slice[i][j][0]*factor;
				p[1] = orig_points_by_slice[i][j][1]*factor;
				p[2] = orig_points_by_slice[i][j][2]*factor;
				transform3(std::get<2>(trajectory[i]), p);
			}
			struct visitor_args data = {};
			data.empty_voxels = &free;
			data.voxel_occupied_by_slice = &voxel_occupied_by_slice;
			data.current_slice = i;
			data.diff = diff;
			walk_voxels(
					std::get<0>(trajectory[i]),
					p,
					voxel_size,
					visitor,
					&data
					);
		}
#ifdef _OPENMP
#pragma omp critical
#endif
		{
			for (struct voxel v : free) {
				free_voxels.insert(v);
			}
			std::cerr << ((done+1)*100.0f/scanorder.size()) << " %\r";
			std::cerr.flush();
			done += 1;
		}
	}
	std::cerr << std::endl;
	clock_gettime(CLOCK_MONOTONIC, &after);
	elapsed = (after.tv_sec - before.tv_sec);
	elapsed += (after.tv_nsec - before.tv_nsec) / 1000000000.0;
	std::cerr << "took: " << elapsed << " seconds" << std::endl;

	std::cerr << "number of freed voxels: " << free_voxels.size() << " (" << (free_voxels.size()*100.0f/voxel_occupied_by_slice.size()) << "% of occupied voxels)" << std::endl;

	if (cluster_size > 1) {
		std::cerr << "clustering voxels" << std::endl;
		clock_gettime(CLOCK_MONOTONIC, &before);
		std::unordered_map<struct voxel, size_t> voxel_to_cluster;
		std::unordered_map<size_t, std::set<struct voxel>> cluster_to_voxel;
		size_t i = 0;
		for (const struct voxel v : free_voxels) {
			std::set<size_t> neighbor_clusters;
			int vradius = 1;
			for (int i = 0-vradius; i <= vradius; ++i) {
				for (int j = 0-vradius; j <= vradius; ++j) {
					for (int k = 0-vradius; k <= vradius; ++k) {
						if (i == 0 && j == 0 && k == 0) {
							continue;
						}
						std::unordered_map<struct voxel, size_t>::iterator it2 = voxel_to_cluster.find(voxel(v.x+i, v.y+j, v.z+k));
						if (it2 != voxel_to_cluster.end()) {
							neighbor_clusters.insert(it2->second);
						}
					}
				}
			}
			if (neighbor_clusters.size() == 0) {
				voxel_to_cluster[v] = i;
				cluster_to_voxel[i].insert(v);
				i += 1;
				continue;
			}
			if (neighbor_clusters.size() == 1) {
				size_t cluster = *(neighbor_clusters.begin());
				voxel_to_cluster[v] = cluster;
				cluster_to_voxel[cluster].insert(v);
				i += 1;
				continue;
			}
			size_t mincluster = *(neighbor_clusters.begin());
			for (size_t cluster : neighbor_clusters) {
				if (cluster == mincluster) {
					continue;
				}
				for (struct voxel v2 : cluster_to_voxel[cluster]) {
					voxel_to_cluster[v2] = mincluster;
					cluster_to_voxel[mincluster].insert(v2);
				}
				cluster_to_voxel.erase(cluster);
			}
			voxel_to_cluster[v] = mincluster;
			cluster_to_voxel[mincluster].insert(v);
			i += 1;
		}
		for (std::pair<size_t, std::set<struct voxel>> p : cluster_to_voxel) {
			if (p.second.size() >= cluster_size) {
				continue;
			}
			for (struct voxel voxel : p.second) {
				free_voxels.erase(voxel);
			}
		}
		std::cerr << "number of free voxels after clustering: " << free_voxels.size() << std::endl;
		clock_gettime(CLOCK_MONOTONIC, &after);
		elapsed = (after.tv_sec - before.tv_sec);
		elapsed += (after.tv_nsec - before.tv_nsec) / 1000000000.0;
		std::cerr << "took: " << elapsed << " seconds" << std::endl;
	}

	std::unordered_map<struct voxel, std::set<size_t>> half_voxels;
	if (!no_subvoxel_accuracy) {
		std::cerr << "calculate half-free voxels" << std::endl;
		clock_gettime(CLOCK_MONOTONIC, &before);
		for (struct voxel v : free_voxels) {
			std::set<struct voxel> neighbor_voxels;
			int vradius = 2;
			for (int i = 0-vradius; i <= vradius; ++i) {
				for (int j = 0-vradius; j <= vradius; ++j) {
					for (int k = 0-vradius; k <= vradius; ++k) {
						if (i == 0 && j == 0 && k == 0) {
							continue;
						}
						struct voxel neighbor = voxel(v.x+i, v.y+j, v.z+k);
						if (free_voxels.find(neighbor) != free_voxels.end()) {
							continue;
						}
						if (voxel_occupied_by_slice.find(neighbor) == voxel_occupied_by_slice.end()) {
							continue;
						}
						for (size_t num : voxel_occupied_by_slice[v]) {
							half_voxels[neighbor].insert(num);
						}
					}
				}
			}
		}
		std::cerr << "number of half-free voxels: " << half_voxels.size() << std::endl;
		clock_gettime(CLOCK_MONOTONIC, &after);
		elapsed = (after.tv_sec - before.tv_sec);
		elapsed += (after.tv_nsec - before.tv_nsec) / 1000000000.0;
		std::cerr << "took: " << elapsed << " seconds" << std::endl;
	}

	std::cerr << "computing connectivity graph..." << std::endl;
	/*
	 * Output a graph where the nodes are the scan ids and edges connect nodes
	 * of scans that share at least one voxel
	 */
	{
		done = 0;
		std::cerr << "0 %\r";
		std::cerr.flush();
		/*
		 * go through all voxels and add all possible pairs of scan
		 * identifiers from a lower id to a higher id to a set
		 */
		std::map<std::pair<size_t, size_t>, size_t> result;
		for (std::pair<struct voxel, std::set<size_t>> el : voxel_occupied_by_slice) {
			/*
			 * the elements of std::set are sorted in ascending order, thus we
			 * do not need to sort them ourselves
			 */
			std::vector<size_t> slices(el.second.begin(), el.second.end());
			for (size_t i = 0; i < slices.size(); ++i) {
				for (size_t j = i+1; j < slices.size(); ++j) {
					result[std::make_pair(slices[i], slices[j])] += 1;
				}
			}
			std::cerr << ((done+1)*100.0f/voxel_occupied_by_slice.size()) << " %\r";
			std::cerr.flush();
			done += 1;
		}
		std::cerr << std::endl;
		size_t num_edges_complete_graph = scanorder.size()*(scanorder.size()-1)/2.0;
		std::cerr << "Scan pairs sharing the same voxel: " << result.size() << std::endl;
		std::cerr << "number edges complete graph: " << num_edges_complete_graph << std::endl;
		std::cerr << "graph G {" << std::endl;
		for (std::pair<std::pair<size_t, size_t>, size_t> el : result) {
			std::cout << "  " << el.first.first << " -- " << el.first.second << " [weight=" << el.second << "];" << std::endl;
		}
		std::cerr << "}" << std::endl;
	}

	std::cerr << "write partitioning" << std::endl;
	clock_gettime(CLOCK_MONOTONIC, &before);

	/*
	 * we use a FILE object instead of an ofstream to write the data because
	 * it gives better performance (probably because it's buffered) and
	 * because of the availability of fprintf
	 */
	FILE *out_static = fopen("scan000.3d", "w");
	FILE *out_dynamic = fopen("scan001.3d", "w");
	done = 0;
	std::cerr << "0 %\r";
	std::cerr.flush();
	for (size_t i : scanorder) {
		std::unordered_map<size_t, DataReflectance>::const_iterator refl_it =
			reflectances_by_slice.find(i);
		for (size_t j = 0; j < points_by_slice[i].size(); ++j) {
			FILE *out;
			struct voxel voxel = voxel_of_point(points_by_slice[i][j],voxel_size);
			if (free_voxels.find(voxel) != free_voxels.end()
					|| (half_voxels.find(voxel) != half_voxels.end() && half_voxels[voxel].find(i) != half_voxels[voxel].end())) {
				out = out_dynamic;
			} else {
				out = out_static;
			}
			double refl = 0;
			if (refl_it != reflectances_by_slice.end()) {
				refl = refl_it->second[j];
			}
			// we print the mantissa with 13 hexadecimal digits because the
			// mantissa for double precision is 52 bits long which is 6.5
			// bytes and thus 13 hexadecimal digits
			//
			// According to
			// http://pubs.opengroup.org/onlinepubs/000095399/functions/printf.html
			// the %a conversion specifier automatically picks the right
			// precision to represent the value exactly.
			fprintf(out, "%a %a %a %a\n",
					points_by_slice[i][j][0],
					points_by_slice[i][j][1],
					points_by_slice[i][j][2],
					//0.0f
					refl
					);
		}
		std::cerr << ((done+1)*100.0f/scanorder.size()) << " %\r";
		std::cerr.flush();
		done += 1;
	}
	std::cerr << std::endl;
	fclose(out_static);
	fclose(out_dynamic);

	FILE *pose_static = fopen("scan000.pose", "w");
	fprintf(pose_static, "0 0 0\n0 0 0\n");
	fclose(pose_static);
	FILE *pose_dynamic = fopen("scan001.pose", "w");
	fprintf(pose_dynamic, "0 0 0\n0 0 0\n");
	fclose(pose_dynamic);

	std::cerr << "write cleaned static scans" << std::endl;
	done = 0;
	std::cerr << "0 %\r";
	std::cerr.flush();
	if (staticdir == "") {
		staticdir = dir + "/static";
	}
	// just mkdir it, ignore errors like EEXIST
	mkdir(staticdir.c_str(), S_IRWXU|S_IRWXG|S_IRWXO);
	for (std::pair<size_t, DataXYZ> element : points_by_slice) {
		size_t i = element.first;
		std::unordered_map<size_t, DataReflectance>::const_iterator refl_it =
			reflectances_by_slice.find(i);
		std::ostringstream out;
		out << staticdir << "/scan" << std::setw(3) << std::setfill('0') << i << ".3d";
		FILE *out_static = fopen(out.str().c_str(), "w");
		if (out_static == NULL) {
			std::cerr << "cannot open" << out.str() << std::endl;
			exit(1);
		}
		for (size_t j = 0; j < element.second.size(); ++j) {
			int ret;
			double refl = 0;
			if (refl_it != reflectances_by_slice.end()) {
				refl = refl_it->second[j];
			}
			struct voxel voxel = voxel_of_point(element.second[j],voxel_size);
			if (free_voxels.find(voxel) == free_voxels.end()
					&& (half_voxels.find(voxel) == half_voxels.end() || half_voxels[voxel].find(i) == half_voxels[voxel].end())) {
				ret = fprintf(out_static, "%a %a %a %a\n",
						orig_points_by_slice[i][j][0],
						orig_points_by_slice[i][j][1],
						orig_points_by_slice[i][j][2],
						//0.0f
						refl
						);
			}
			if (ret < 0) {
				std::cerr << "failed to write to " << out.str() << std::endl;
				exit(1);
			}
		}
		fclose(out_static);
		std::ostringstream out_pose;
		out_pose << staticdir << "/scan" << std::setw(3) << std::setfill('0') << i << ".pose";
		FILE *pose = fopen(out_pose.str().c_str(), "w");
		int ret = fprintf(pose, "%.17f %.17f %.17f\n%.17f %.17f %.17f\n",
				std::get<0>(trajectory[i])[0],
				std::get<0>(trajectory[i])[1],
				std::get<0>(trajectory[i])[2],
				std::get<1>(trajectory[i])[0]*180/M_PI,
				std::get<1>(trajectory[i])[1]*180/M_PI,
				std::get<1>(trajectory[i])[2]*180/M_PI
				);
		if (ret < 0) {
			std::cerr << "failed to write to " << out_pose.str() << std::endl;
			exit(1);
		}
		fclose(pose);
		std::cerr << ((done+1)*100.0f/points_by_slice.size()) << " %\r";
		std::cerr.flush();
		done += 1;
	}
	std::cerr << std::endl;

	std::cerr << "write masks" << std::endl;
	done = 0;
	std::cerr << "0 %\r";
	std::cerr.flush();

	if (maskdir == "") {
		maskdir = dir + "/pplremover";
	}
	// just mkdir it, ignore errors like EEXIST
	mkdir(maskdir.c_str(), S_IRWXU|S_IRWXG|S_IRWXO);
	for (std::pair<size_t, DataXYZ> element : points_by_slice) {
		std::ostringstream out;
		out << maskdir << "/scan" << std::setw(3) << std::setfill('0') << element.first << ".mask";
		FILE *out_mask = fopen(out.str().c_str(), "w");
		if (out_mask == NULL) {
			std::cerr << "cannot open" << out.str() << std::endl;
			exit(1);
		}
		for (size_t i = 0; i < element.second.size(); ++i) {
			int ret;
			struct voxel voxel = voxel_of_point(element.second[i],voxel_size);
			if (free_voxels.find(voxel) != free_voxels.end()
					|| (half_voxels.find(voxel) != half_voxels.end() && half_voxels[voxel].find(element.first) != half_voxels[voxel].end())) {
				ret = fprintf(out_mask, "1\n");
			} else {
				ret = fprintf(out_mask, "0\n");
			}
			if (ret < 0) {
				std::cerr << "failed to write to " << out.str() << std::endl;
				exit(1);
			}
		}
		fclose(out_mask);
		std::cerr << ((done+1)*100.0f/points_by_slice.size()) << " %\r";
		std::cerr.flush();
		done += 1;
	}
	std::cerr << std::endl;

	clock_gettime(CLOCK_MONOTONIC, &after);
	elapsed = (after.tv_sec - before.tv_sec);
	elapsed += (after.tv_nsec - before.tv_nsec) / 1000000000.0;
	std::cerr << "took: " << elapsed << " seconds" << std::endl;

	std::cerr << "done" << std::endl;

	for (size_t idx = 0; idx < scanorder.size(); ++idx) {
		size_t i = scanorder[idx];
#ifdef WITH_MMAP_SCAN
		if (!cachedir.empty()) {
			int ret;
			ret = munmap(orig_points_by_slice[i].get_raw_pointer(), orig_points_by_slice[i].size());
			if (ret != 0) {
				throw std::runtime_error("cannot munmap");
			}
			// since we called unlink() before, this also deletes the file for good
			ret = close(mmap_fds[i]);
			if (ret != 0) {
				throw std::runtime_error("cannot close");
			}
		} else {
#endif
			delete orig_points_by_slice[i].get_raw_pointer();
#ifdef WITH_MMAP_SCAN
		}
#endif
	}

	Scan::closeDirectory();

	return 0;
}
