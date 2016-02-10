/*
 * collision_sphere
 *
 * Copyright (C) Johannes Schauer
 *
 * Released under the GPL version 3.
 *
 */

#ifdef _WIN32
#include <direct.h>
#endif

#include "slam6d/scan.h"
#include "slam6d/frame.h"
#include "slam6d/globals.icc"
#include "slam6d/kdIndexed.h"
#include "scanio/scan_io.h"

#include <time.h>
#include <sys/stat.h>
#include <sys/types.h>

#include <boost/program_options.hpp>
namespace po = boost::program_options;


#ifdef WITH_CUDA
#include "cuda/tools.h"
#include "cuda/grid.h"
#endif //WITH_CUDA


#ifdef _MSC_VER
#define strcasecmp _stricmp
#define strncasecmp _strnicmp
#endif

enum collision_method { CTYPE1, CTYPE2, CTYPE3 };
enum penetrationdepth_method { PDTYPE1, PDTYPE2 };

/*
 * validates input type specification
 */
void validate(boost::any& v, const std::vector<std::string>& values,
    IOType*, int) {
    if (values.size() == 0)
        throw std::runtime_error("Invalid model specification");
    string arg = values.at(0);
    try {
        v = formatname_to_io_type(arg.c_str());
    } catch (...) { // runtime_error
        throw std::runtime_error("Format " + arg + " unknown.");
    }
}

void validate(boost::any& v, const std::vector<std::string>& values,
        collision_method*, int) {
    if (values.size() == 0)
        throw std::runtime_error("Invalid model specification");
    string arg = values.at(0);
    if (strcasecmp(arg.c_str(), "type1") == 0) v = CTYPE1;
    else if (strcasecmp(arg.c_str(), "type2") == 0) v = CTYPE2;
    else if (strcasecmp(arg.c_str(), "type3") == 0) v = CTYPE3;
    else throw std::runtime_error(std::string("collision method ")
            + arg + std::string(" is unknown"));
}

void validate(boost::any& v, const std::vector<std::string>& values,
        penetrationdepth_method*, int) {
    if (values.size() == 0)
        throw std::runtime_error("Invalid model specification");
    string arg = values.at(0);
    if (strcasecmp(arg.c_str(), "type1") == 0) v = PDTYPE1;
    else if (strcasecmp(arg.c_str(), "type2") == 0) v = PDTYPE2;
    else throw std::runtime_error(std::string("penetration depth method ")
            + arg + std::string(" is unknown"));
}

void parse_options(int argc, char **argv, IOType &iotype, string &dir,
        double &radius, bool &calcdistances, collision_method &cmethod,
        penetrationdepth_method &pdmethod, bool &use_cuda, int &cuda_device)
{
    po::options_description generic("Generic options");
    generic.add_options()
        ("help,h", "output this help message");

    po::options_description input("Input options");
    input.add_options()
        ("format,f", po::value<IOType>(&iotype)->default_value(UOS),
         "using shared library <arg> for input. (chose F from {uos, uos_map, "
         "uos_rgb, uos_frames, uos_map_frames, old, rts, rts_map, ifp, "
         "riegl_txt, riegl_rgb, riegl_bin, zahn, ply})");

    po::options_description prog("Program specific options");
    prog.add_options()
        ("radius,r", po::value<double>(&radius)->default_value(10),
         "radius of sphere")
        ("calcdistances,d", po::value<bool>(&calcdistances)->zero_tokens(),
         "calculate penetration distance")
        ("collisionmethod,c", po::value<collision_method>(&cmethod)->default_value(CTYPE1),"CPU collision method")
        ("penetrationdepthmethod,p", po::value<penetrationdepth_method>(&pdmethod)->default_value(PDTYPE1))
        ("usecuda,C", po::value<bool>(&use_cuda)->zero_tokens(),"Use NVIDIA CUDA")
		("device,D", po::value<int>(&cuda_device)->default_value(0));

    po::options_description hidden("Hidden options");
    hidden.add_options()
        ("input-dir", po::value<string>(&dir), "input dir");

    // all options
    po::options_description all;
    all.add(generic).add(input).add(prog).add(hidden);

    // options visible with --help
    po::options_description cmdline_options;
    cmdline_options.add(generic).add(input).add(prog);

    // positional argument
    po::positional_options_description pd;
    pd.add("input-dir", 1);

    // process options
    po::variables_map vm;
    po::store(po::command_line_parser(argc, argv).
              options(all).positional(pd).run(), vm);
    po::notify(vm);

    // display help
    if (vm.count("help")) {
        cout << cmdline_options;
        cout << "\nExample usage:\n"
             << "\t" << argv[0] << " --radius=10 -f xyzr ./dir\n";
        cout << "The directory ./dir must contain the model as the first scan and\n";
        cout << "the environment as the second scan. The frames file of the first\n";
        cout << "scan is the trajectory.";
        exit(0);
    }

    if (!vm.count("input-dir")) {
        cout << "you have to specify an input directory" << endl;
        exit(1);
    }
    if (dir[dir.length()-1] != '/') dir = dir + "/";
}

void createdirectory(string segdir)
{
    int success;
#ifdef _WIN32
    success = _mkdir(segdir.c_str());
#else
    success = mkdir(segdir.c_str(), S_IRWXU|S_IRWXG|S_IRWXO);
#endif
    if (success == 0 || errno == EEXIST) {
        cout << "Writing colliding points to " << segdir << endl;
    } else {
        cerr << "Creating directory " << segdir << " failed" << endl;
        exit(1);
    }
}

std::vector<Frame> read_trajectory(string filename)
{
    std::ifstream file(filename.c_str());
    if (!file.good()) {
        cout << "can't open " << filename << " for reading" << endl;
        exit(1);
    }
    std::vector<Frame> positions;
    double transformation[16];
    unsigned int type;
    std::string line;
    while (std::getline(file, line)) {
        std::istringstream iss(line);
        iss >> transformation >> type;
        positions.push_back(Frame(transformation, type));
    }
    return positions;
}

void write_xyzr(DataXYZ &points, DataReflectance &refl, string &dir,
        std::vector<bool> const &colliding,
        std::vector<float> const &dist_colliding, double radius)
{
	// sanity checks
    if (points.size() != refl.size() || points.size() != colliding.size()) {
		cerr << "differing data lengths: " << points.size() << " vs. " << refl.size() << " vs. " << colliding.size() << endl;
		return;
	}
	// find the minimum and maximum for reflectance and penetration depth
	double min_refl, max_refl, min_pd, max_pd;
	min_refl = min_pd = std::numeric_limits<double>::max();
	max_refl = max_pd = std::numeric_limits<double>::min();
	for (size_t i = 0; i < refl.size(); ++i) {
		if (refl[i] < min_refl)
			min_refl = refl[i];
		if (refl[i] > max_refl)
			max_refl = refl[i];
	}
	for (size_t i = 0; i < dist_colliding.size(); ++i) {
		if (dist_colliding[i] < min_pd)
			min_pd = dist_colliding[i];
		if (dist_colliding[i] > max_pd)
			max_pd = dist_colliding[i];
	}
	cout << "min refl: " << min_refl << endl;
	cout << "max refl: " << max_refl << endl;
	cout << "min pd: " << min_pd << endl;
	cout << "max pd: " << max_pd << endl;
    cerr << "writing colliding points to " << dir << "scan002.xyz" << endl;
    ofstream fcolliding((dir + "scan002.xyz").c_str());
    cerr << "writing non-colliding points to " << dir << "scan003.xyz" << endl;
    ofstream fnoncolliding((dir + "scan003.xyz").c_str());
	double r;
    for (size_t i = 0, j=0; i < points.size(); ++i) {
        if (colliding[i]) {
			// Convert from left handed to right handed and normalize
			// penetration distance to values -1 <= r < 0.
			// Since penetration distances always start at and are always
			// greater than 0 we don't need to shift.
			// We cap penetration distances at the configured search radius.
			r = fmax(-1.0*(dist_colliding[j]/radius),-1.0);
            fcolliding << points[i][2] << " " << -points[i][0] << " " << points[i][1] << " " << r << endl;
            j++;
        } else {
			// convert from left handed to right handed and normalize
			// reflectance to values 0 <= r < 1
			r = (refl[i]-min_refl)/(max_refl-min_refl+1);
            fnoncolliding << points[i][2] << " " << -points[i][0] << " " << points[i][1] << " " << r << endl;
        }
    }
    fcolliding.close();
    fnoncolliding.close();
    ofstream pcolliding((dir + "scan002.pose").c_str());
    pcolliding << "0 0 0" << endl << "0 0 0" << endl;
    pcolliding.close();
    ofstream pnoncolliding((dir + "scan003.pose").c_str());
    pnoncolliding << "0 0 0" << endl << "0 0 0" << endl;
    pnoncolliding.close();
}

/*
std::vector<Point> read_plymodel(string &pointmodelpath)
{
    IOType m_type = PLY;
    ScanIO* sio = ScanIO::getScanIO(m_type);
    PointFilter filter;
    vector<double> xyz;
    vector<unsigned char> rgb;
    vector<float> reflectance;
    sio->readScan(pointmodelpath.c_str(), "model", filter, &xyz, &rgb, &reflectance, NULL, NULL, NULL, NULL);
    vector<Point> points;
    for(std::vector<double>::iterator it = xyz.begin(); it != xyz.end();) {
        double x = *it; ++it;
        double y = *it; ++it;
        double z = *it; ++it;
        points.push_back(Point(x,y,z));
    }
    return points;
}
*/

void fill_colliding(std::vector<bool> &allcolliding, std::vector<size_t> const &newindices)
{
    for(auto it : newindices) {
        allcolliding[it] = true;
    }
}

size_t handle_pointcloud(std::vector<Point> &pointmodel, DataXYZ &environ,
                       std::vector<Frame> const &trajectory,
                       std::vector<bool> &colliding,
                       double radius, collision_method cmethod)
{
    /* build a KDtree from this scan */
    cerr << "reading environment..." << endl;
    double** pa = new double*[environ.size()];
    size_t i;
    for (i = 0; i < environ.size(); ++i) {
        pa[i] = new double[3];
        pa[i][0] = environ[i][0];
        pa[i][1] = environ[i][1];
        pa[i][2] = environ[i][2];
    }
    cerr << "environment: " << i << endl;
    cerr << "building kd tree..." << endl;
    KDtreeIndexed t(pa, environ.size());
    /* initialize variables */
    int thread_num = 0; // add omp later
    double sqRad2 = radius*radius;
    cerr << "computing collisions..." << endl;
    time_t before = time(NULL);
    int end;
    i = 0;
    switch (cmethod) {
        case CTYPE1:
            end = trajectory.size();
            for(const auto &it2 : trajectory) {
                cerr << (i*100.0)/end << " %\r";
                cerr.flush();
                for(const auto &it : pointmodel) {
                    double point1[3] = {it.x, it.y, it.z};
                    transform3(it2.transformation, point1);
                    vector<size_t> collidingsphere = t.fixedRangeSearch(point1, sqRad2, thread_num);
                    fill_colliding(colliding, collidingsphere);
                }
                i++;
            }
            break;
        case CTYPE2:
            end = pointmodel.size();
            // we iterate over points instead of points on the trajectory so
            // that we can reuse the previous transformation of the same point
            for(const auto &it : pointmodel) {
                cerr << (i*100.0)/end << " %\r";
                cerr.flush();
                auto it2 = trajectory.begin();
                double point1[3], point2[3];
                point1[0] = it.x;
                point1[1] = it.y;
                point1[2] = it.z;
                transform3(it2->transformation, point1);
                ++it2;
                for (; it2 < trajectory.end(); ++it2) {
                    point2[0] = it.x;
                    point2[1] = it.y;
                    point2[2] = it.z;
                    transform3(it2->transformation, point2);
                    vector<size_t> collidingsegment = t.segmentSearch_all(point1, point2, sqRad2, thread_num);
                    fill_colliding(colliding, collidingsegment);
                    point1[0] = point2[0];
                    point1[1] = point2[1];
                    point1[2] = point2[2];
                }
                i++;
            }
            break;
        default:
            throw std::runtime_error("impossible");
    }
    size_t num_colliding = 0;
    // the actual implementation of std::vector<bool> requires us to use the
    // proxy iterator pattern with &&...
    for (i = 0; i < environ.size(); ++i) {
        if (colliding[i]) {
            num_colliding++;
        }
    }
    time_t after = time(NULL);
    cerr << "colliding: " << num_colliding << endl;
    cerr << "took: " << difftime(after, before) << " seconds" << endl;
    for (i = 0; i < environ.size(); ++i) {
        delete[] pa[i];
    }
    delete[] pa;
    return num_colliding;
}

#ifdef WITH_CUDA

///////////////////////////////////////////////////////////////////////////////
// Creates a 4x4 rotation matrix, takes radians NOT degrees
void rotationMatrix(float angle, float x, float y, float z, double mMatrix[16])
{
    float vecLength, sinSave, cosSave, oneMinusCos;
    float xx, yy, zz, xy, yz, zx, xs, ys, zs;

    // If NULL vector passed in, this will blow up...
    if(x==0.0f && y==0.0f && z==0.0f)
    {
		static double identity[16]={ 1.0f, 0.0f, 0.0f, 0.0f,
                                     0.0f, 1.0f, 0.0f, 0.0f,
                                     0.0f, 0.0f, 1.0f, 0.0f,
                                     0.0f, 0.0f, 0.0f, 1.0f };

		memcpy(mMatrix, identity, 16*sizeof(double));
        return;
    }

    // Scale vector
    vecLength=(float)sqrt( x*x+y*y+z*z );

    // Rotation matrix is normalized
    x /=vecLength;
    y /=vecLength;
    z /=vecLength;
        
    sinSave=(float)sin(angle);
    cosSave=(float)cos(angle);
    oneMinusCos=1.0f-cosSave;

    xx=x*x;
    yy=y*y;
    zz=z*z;
    xy=x*y;
    yz=y*z;
    zx=z*x;
    xs=x*sinSave;
    ys=y*sinSave;
    zs=z*sinSave;

    mMatrix[0]=(oneMinusCos*xx)+cosSave;
    mMatrix[4]=(oneMinusCos*xy)-zs;
    mMatrix[8]=(oneMinusCos*zx)+ys;
    mMatrix[12]=0.0f;

    mMatrix[1]=(oneMinusCos*xy)+zs;
    mMatrix[5]=(oneMinusCos*yy)+cosSave;
    mMatrix[9]=(oneMinusCos*yz)-xs;
    mMatrix[13]=0.0f;

    mMatrix[2]=(oneMinusCos*zx)-ys;
    mMatrix[6]=(oneMinusCos*yz)+xs;
    mMatrix[10]=(oneMinusCos*zz)+cosSave;
    mMatrix[14]=0.0f;

    mMatrix[3]=0.0f;
    mMatrix[7]=0.0f;
    mMatrix[11]=0.0f;
    mMatrix[15]=1.0f;
}
void multiplyMatrix(const double* m1, const double* m2, double* mProduct )
{
	mProduct[0]=m1[0]*m2[0]+m1[4]*m2[1]+m1[8]*m2[2]+m1[12]*m2[3];
	mProduct[4]=m1[0]*m2[4]+m1[4]*m2[5]+m1[8]*m2[6]+m1[12]*m2[7];
	mProduct[8]=m1[0]*m2[8]+m1[4]*m2[9]+m1[8]*m2[10]+m1[12]*m2[11];
	mProduct[12]=m1[0]*m2[12]+m1[4]*m2[13]+m1[8]*m2[14]+m1[12]*m2[15];

	mProduct[1]=m1[1]*m2[0]+m1[5]*m2[1]+m1[9]*m2[2]+m1[13]*m2[3];
	mProduct[5]=m1[1]*m2[4]+m1[5]*m2[5]+m1[9]*m2[6]+m1[13]*m2[7];
	mProduct[9]=m1[1]*m2[8]+m1[5]*m2[9]+m1[9]*m2[10]+m1[13]*m2[11];
	mProduct[13]=m1[1]*m2[12]+m1[5]*m2[13]+m1[9]*m2[14]+m1[13]*m2[15];

	mProduct[2]=m1[2]*m2[0]+m1[6]*m2[1]+m1[10]*m2[2]+m1[14]*m2[3];
	mProduct[6]=m1[2]*m2[4]+m1[6]*m2[5]+m1[10]*m2[6]+m1[14]*m2[7];
	mProduct[10]=m1[2]*m2[8]+m1[6]*m2[9]+m1[10]*m2[10]+m1[14]*m2[11];
	mProduct[14]=m1[2]*m2[12]+m1[6]*m2[13]+m1[10]*m2[14]+m1[14]*m2[15];

	mProduct[3]=m1[3]*m2[0]+m1[7]*m2[1]+m1[11]*m2[2]+m1[15]*m2[3];
	mProduct[7]=m1[3]*m2[4]+m1[7]*m2[5]+m1[11]*m2[6]+m1[15]*m2[7];
	mProduct[11]=m1[3]*m2[8]+m1[7]*m2[9]+m1[11]*m2[10]+m1[15]*m2[11];
	mProduct[15]=m1[3]*m2[12]+m1[7]*m2[13]+m1[11]*m2[14]+m1[15]*m2[15];     
}

size_t cuda_handle_pointcloud(int cuda_device, std::vector<Point> &pointmodel, DataXYZ &environ,
		std::vector<Frame> const &trajectory,
		std::vector<bool> &colliding,
		double radius, collision_method cmethod)
{
	const clock_t begin_time = clock();
	
	//colliding
	for(unsigned int i=0;i<colliding.size();++i)
	{
		colliding[i]=false;
	}
	
	
	CuGrid grid(cuda_device);
	
	
	double *env_xyz=new double[environ.size()*3];
	//Copy data
	for(unsigned int i=0;i<environ.size();++i)
	{
		env_xyz[3*i+0]=-environ[i][1];
		env_xyz[3*i+1]=environ[i][2];
		env_xyz[3*i+2]=environ[i][0];
	}
	grid.SetD(env_xyz,environ.size());	//xyz swaped
	delete[] env_xyz;
	
	//grid.SetD((double*)environ.get_raw_pointer(),environ.size());
	
	double *tmp_xyz=new double[pointmodel.size()*3];
	//Copy data
	for(unsigned int i=0;i<pointmodel.size();++i)
	{
		tmp_xyz[3*i+0]=-pointmodel[i].y;	//xyz swaped
		tmp_xyz[3*i+1]=pointmodel[i].z;
		tmp_xyz[3*i+2]=pointmodel[i].x;
		/*
		tmp_xyz[3*i+0]=pointmodel[i].x;
		tmp_xyz[3*i+1]=pointmodel[i].y;
		tmp_xyz[3*i+2]=pointmodel[i].z;*/
	}
	grid.SetM(tmp_xyz,pointmodel.size());
	
	
	grid.SetRadius(radius);
	
	for(unsigned int i=0;i<trajectory.size();++i)
	{
		printf("Trajectory: %d / %lu\n",i,trajectory.size());
	
		//Create scale matrix
		double mat_scale[16];
		double mat[16];
		
		for(int i=0;i<16;++i)
		{
			mat_scale[i]=0;
		}
		mat_scale[0]=mat_scale[5]=mat_scale[10]=mat_scale[15]=grid.params.scale;
		
		multiplyMatrix(trajectory[i].transformation,mat_scale,mat);
		
		grid.SetM(tmp_xyz,pointmodel.size());
		grid.TransformM(mat);
		
		vector<int> output=grid.fixedRangeSearch();
		
		//Fill colliding
		for(unsigned int i=0;i<colliding.size() && i<output.size();++i)
		{
			if(output[i] != -1)
				colliding[i]=true;
		}
		
	}
	
	
	delete[] tmp_xyz;
	
	
	size_t num_colliding = 0;
	// the actual implementation of std::vector<bool> requires us to use the
	// proxy iterator pattern with &&...
	for (unsigned int i = 0; i < environ.size(); ++i)
	{
		if (colliding[i])
		{
			num_colliding++;
		}
	}
	
	
	float time = float( clock () - begin_time ) /  CLOCKS_PER_SEC;
	printf("colliding: %lu / %u\n",num_colliding,environ.size());
	cerr << "took: " << time << " seconds" << endl;
	
	
	/*
	FILE *f=fopen("output-red20-coll.asc","w");
	if(f)
	{
		printf("Saving output-red20.asc\n");
		for(int i=0;i<grid.GetDsize();i+=20)
		{
			if(colliding[i])
				fprintf(f,"%f %f %f 255 0 0\n",grid.GetD()[3*i+0],grid.GetD()[3*i+1],grid.GetD()[3*i+2]);
			else
				fprintf(f,"%f %f %f 0 128 0\n",grid.GetD()[3*i+0],grid.GetD()[3*i+1],grid.GetD()[3*i+2]);
		}
		fclose(f);
	}*/
	
	return num_colliding;
}
#endif //WITH_CUDA
void calculate_collidingdist(DataXYZ &environ,
                             std::vector<bool> const &colliding,
                             size_t num_colliding,
                             std::vector<float> &dist_colliding)
{
    /* build a kdtree for the non-colliding points */
    cerr << "reading environment..." << endl;
    size_t num_noncolliding = environ.size() - num_colliding;
    double** pa = new double*[num_noncolliding];
    size_t* idxmap = new size_t[num_noncolliding];
    for (size_t i = 0, j = 0; i < environ.size(); ++i) {
        if (colliding[i]) {
            continue;
        }
        pa[j] = new double[3];
        pa[j][0] = environ[i][0];
        pa[j][1] = environ[i][1];
        pa[j][2] = environ[i][2];
        idxmap[j] = i;
        j++;
    }
    cerr << "noncolliding: " << num_noncolliding << endl;
    cerr << "building kd tree..." << endl;
    KDtreeIndexed t(pa, num_noncolliding);
    int thread_num = 0; // add omp later
    cerr << "computing distances..." << endl;
    time_t before = time(NULL);
    int j = 0;
    for (size_t i = 0; i < environ.size(); ++i) {
        if (!colliding[i]) {
            continue;
        }
        cerr << (j*100.0)/num_colliding << " %\r";
        cerr.flush();
        double point1[3] = {environ[i][0], environ[i][1], environ[i][2]};
        // for this colliding point, find the closest non-colliding one
        size_t c = t.FindClosest(point1, 1000000, thread_num);
        /*if (colliding[c]) {
            cerr << "result cannot be part of colliding points" << endl;
            cerr << environ[i][0] << " " << environ[i][1] << " " << environ[i][2] << endl;
            cerr << environ[c][0] << " " << environ[c][1] << " " << environ[c][2] << endl;
            exit(1);
        }*/
        c = idxmap[c];
        double point2[3] = {environ[c][0], environ[c][1], environ[c][2]};
        dist_colliding[j] = sqrt(Dist2(point1, point2));
        j++;
    }
    time_t after = time(NULL);
    cerr << "took: " << difftime(after, before) << " seconds" << endl;
    for (size_t i = 0; i < num_noncolliding; ++i) {
        delete[] pa[i];
    }
    delete[] pa;
    delete[] idxmap;
}

void calculate_collidingdist2(std::vector<Point> &pointmodel, DataXYZ &environ,
                             std::vector<Frame> const &trajectory,
                             std::vector<bool> const &colliding,
                             size_t num_colliding,
                             std::vector<float> &dist_colliding,
                             double radius)
{
    /* build a kdtree for colliding points */
    cerr << "reading environment..." << endl;
    double** pa = new double*[num_colliding];
    for (size_t i = 0, j = 0; i < environ.size(); ++i) {
        if (!colliding[i]) {
            continue;
        }
        pa[j] = new double[3];
        pa[j][0] = environ[i][0];
        pa[j][1] = environ[i][1];
        pa[j][2] = environ[i][2];
        j++;
    }
    cerr << "colliding: " << num_colliding << endl;
    cerr << "building kd tree..." << endl;
    KDtreeIndexed t(pa, num_colliding);
    double sqRad2 = radius*radius;
    int thread_num = 0; // add omp later
    cerr << "computing distances..." << endl;
    time_t before = time(NULL);
    int i = 0;
    int end = trajectory.size();
    for(const auto &it2 : trajectory) {
        cerr << (i*100.0)/end << " %\r";
        cerr.flush();
        for(const auto &it : pointmodel) {
            double point1[3] = {it.x, it.y, it.z};
            // the second point is the projection of the first point to the
            // y-axis of the model
            double point2[3] = {0, it.y, 0};
            transform3(it2.transformation, point1);
            transform3(it2.transformation, point2);
            size_t c1 = t.segmentSearch_1NearestPoint(point1, point2, sqRad2, thread_num);
            if (c1 >= std::numeric_limits<size_t>::max()) {
				// found nothing
				continue;
			}
			double dist2 = Dist2(point1, pa[c1]);
			// now get all points around this closest point to mark them
			// with the same penetration distance
			vector<size_t> closestsphere = t.fixedRangeSearch(pa[c1], sqRad2, thread_num);
			for (const auto &it3 : closestsphere) {
				if (dist2 < dist_colliding[it3]) {
					dist_colliding[it3] = dist2;
				}
			}
        }
        i++;
    }
	// turn the squared distances into normal ones
    for (size_t i = 0; i < dist_colliding.size(); ++i) {
        dist_colliding[i] = sqrt(dist_colliding[i]);
    }
    time_t after = time(NULL);
    cerr << "took: " << difftime(after, before) << " seconds" << endl;
    for (size_t i = 0; i < num_colliding; ++i) {
        delete[] pa[i];
    }
    delete[] pa;
}

int main(int argc, char **argv)
{
    // commandline arguments
    string dir;
    IOType iotype;
    double radius;
    bool calcdistances;
    collision_method cmethod;
    penetrationdepth_method pdmethod;
	bool use_cuda=0;
	int cuda_device=0;
	
    parse_options(argc, argv, iotype, dir, radius, calcdistances, cmethod, pdmethod, use_cuda, cuda_device);

    // read scan 0 (model) and 1 (environment) without scanserver
    Scan::openDirectory(false, dir, iotype, 0, 1);

    if(Scan::allScans.size() == 0) {
      cerr << "No scans found. Did you use the correct format?" << endl;
      exit(-1);
    }

    // trajectory is the *.frames file of the model
    string trajectoryfn = dir+"scan000.frames";

    cerr << "reading trajectory from " << trajectoryfn << endl;
    // read trajectory in *.3d file format
    std::vector<Frame> trajectory = read_trajectory(trajectoryfn);

    ScanVector::iterator it = Scan::allScans.begin();

    // if matching against pointcloud, treat the first scan as the model
    if(Scan::allScans.size() != 2) {
        cerr << "must supply two scans, the model and the environment in that order" << endl;
        exit(-1);
    }
    DataXYZ model(it[0]->get("xyz"));
    DataXYZ environ(it[1]->get("xyz"));
    DataReflectance refl(it[1]->get("reflectance"));
    std::vector<bool> colliding;
    colliding.resize(environ.size(), false); // by default, nothing collides
    cerr << "reading model..." << endl;
    vector<Point> pointmodel;
    for(unsigned int j = 0; j < model.size(); j++) {
        pointmodel.push_back(Point(model[j][0], model[j][1], model[j][2]));
    }
    cerr << "model: " << pointmodel.size() << endl;
	
    size_t num_colliding = 0;
	if(use_cuda)
	{
#ifdef WITH_CUDA
		
		if(!ValidCUDADevice(cuda_device))
		{
			cerr << "No such CUDA device: " << cuda_device << endl;
			PrintCudaInfo();
			exit(-1);
		}
		
		cerr << "built with CUDA; use_cuda selected; will use CUDA\n";
		num_colliding = cuda_handle_pointcloud(cuda_device,pointmodel, environ, trajectory, colliding, radius, cmethod);
		// FIXME: when there is no CUDA devices use CPU version
#else
		cerr << "built without CUDA; use_cuda selected\n";
		cerr << "EXIT\n";
		return -1;
#endif
	}
	else
	{
#ifdef WITH_CUDA
		cerr << "built with CUDA; will NOT use CUDA\n";
#else
		cerr << "built without CUDA; will NOT use CUDA\n";
#endif
	
		if (cmethod == CTYPE3) {
			num_colliding = environ.size();
			for (unsigned int i = 0; i < environ.size(); ++i) {
				colliding[i] = true;
			}
		} else {
			num_colliding = handle_pointcloud(pointmodel, environ, trajectory, colliding, radius, cmethod);
		}
	}
    if (num_colliding == 0) {
        cerr << "nothing collides" << endl;
        exit(0);
    }
    std::vector<float> dist_colliding;
	// FIXME: make the maximum colliding distance configurable
    dist_colliding.resize(num_colliding, 1000);
    if (calcdistances) {
        switch (pdmethod) {
            case PDTYPE1:
                calculate_collidingdist(environ, colliding, num_colliding, dist_colliding);
                break;
            case PDTYPE2:
                calculate_collidingdist2(pointmodel, environ, trajectory, colliding, num_colliding, dist_colliding, radius);
                break;
        }
    }
    write_xyzr(environ, refl, dir, colliding, dist_colliding, radius);
}
