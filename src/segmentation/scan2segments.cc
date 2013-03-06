/*
 * scan_red implementation
 *
 * Copyright (C) Johannes Schauer
 *
 * Released under the GPL version 3.
 *
 */

#include "slam6d/scan.h"
#include "slam6d/globals.icc"

#include <string>
using std::string;

#include <iostream>
using std::cout;
using std::endl;

#include <algorithm>

#include <boost/program_options.hpp>
namespace po = boost::program_options;

#include "slam6d/fbr/panorama.h"

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/legacy/legacy.hpp>

#include <sys/stat.h>
#include <sys/types.h>

#ifdef _MSC_VER
#include <direct.h>
#define mkdir(path,mode) _mkdir (path)
#endif

enum image_type {M_RANGE, M_INTENSITY};

enum segment_method {THRESHOLD, ADAPTIVE_THRESHOLD, PYR_MEAN_SHIFT, PYR_SEGMENTATION, WATERSHED};

/* Function used to check that 'opt1' and 'opt2' are not specified
   at the same time. */
void conflicting_options(const po::variables_map & vm,
                         const char *opt1, const char *opt2)
{
    if (vm.count(opt1) && !vm[opt1].defaulted()
        && vm.count(opt2) && !vm[opt2].defaulted())
        throw std::logic_error(string("Conflicting options '")
                               + opt1 + "' and '" + opt2 + "'.");
}

/* Function used to check that if 'for_what' is specified, then
   'required_option' is specified too. */
void option_dependency(const po::variables_map & vm,
                       const char *for_what, const char *required_option)
{
    if (vm.count(for_what) && !vm[for_what].defaulted())
        if (vm.count(required_option) == 0
            || vm[required_option].defaulted())
            throw std::logic_error(string("Option '") + for_what +
                                   "' requires option '" +
                                   required_option + "'.");
}

/*
 * validates panorama method specification
 */
namespace fbr {
    void validate(boost::any& v, const std::vector<std::string>& values,
                  projection_method*, int) {
        if (values.size() == 0)
            throw std::runtime_error("Invalid model specification");
        string arg = values.at(0);
        if(strcasecmp(arg.c_str(), "EQUIRECTANGULAR") == 0) v = EQUIRECTANGULAR;
        else if(strcasecmp(arg.c_str(), "CYLINDRICAL") == 0) v = CYLINDRICAL;
        else if(strcasecmp(arg.c_str(), "MERCATOR") == 0) v = MERCATOR;
        else if(strcasecmp(arg.c_str(), "RECTILINEAR") == 0) v = RECTILINEAR;
        else if(strcasecmp(arg.c_str(), "PANNINI") == 0) v = PANNINI;
        else if(strcasecmp(arg.c_str(), "STEREOGRAPHIC") == 0) v = STEREOGRAPHIC;
        else if(strcasecmp(arg.c_str(), "ZAXIS") == 0) v = ZAXIS;
        else if(strcasecmp(arg.c_str(), "CONIC") == 0) v = CONIC;
        else throw std::runtime_error(std::string("projection method ") + arg + std::string(" is unknown"));
    }
}

/*
 * validates segmentation method specification
 */
void validate(boost::any& v, const std::vector<std::string>& values,
        segment_method*, int) {
    if (values.size() == 0)
        throw std::runtime_error("Invalid model specification");
    string arg = values.at(0);
    if(strcasecmp(arg.c_str(), "THRESHOLD") == 0) v = THRESHOLD;
    else if(strcasecmp(arg.c_str(), "ADAPTIVE_THRESHOLD") == 0) v = ADAPTIVE_THRESHOLD;
    else if(strcasecmp(arg.c_str(), "PYR_MEAN_SHIFT") == 0) v = PYR_MEAN_SHIFT;
    else if(strcasecmp(arg.c_str(), "PYR_SEGMENTATION") == 0) v = PYR_SEGMENTATION;
    else if(strcasecmp(arg.c_str(), "WATERSHED") == 0) v = WATERSHED;
    else throw std::runtime_error(std::string("segmentation method ") + arg + std::string(" is unknown"));
}

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

void segmentation_option_dependency(const po::variables_map & vm, segment_method stype, const char *option)
{
    if (vm.count("segment") && vm["segment"].as<segment_method>() == stype) {
        if (!vm.count(option)) {
            throw std::logic_error (string("this segmentation option needs ")+option+" to be set");
        }
    }
}

void segmentation_option_conflict(const po::variables_map & vm, segment_method stype, const char *option)
{
    if (vm.count("segment") && vm["segment"].as<segment_method>() == stype) {
        if (vm.count(option)) {
            throw std::logic_error (string("this segmentation option is incompatible with ")+option);
        }
    }
}
/*
 * parse commandline options, fill arguments
 */
void parse_options(int argc, char **argv, int &start, int &end,
        bool &scanserver, image_type &itype, int &width, int &height,
        fbr::projection_method &ptype, string &dir, IOType &iotype,
        int &maxDist, int &minDist, int &nImages, int &pParam, bool &logarithm,
        float &cutoff, segment_method &stype, string &marker, bool &dump_pano,
        bool &dump_seg, double &thresh, int &maxlevel, int &radius,
        double &pyrlinks, double &pyrcluster, int &pyrlevels)
{
    po::options_description generic("Generic options");
    generic.add_options()
        ("help,h", "output this help message");

    po::options_description input("Input options");
    input.add_options()
        ("start,s", po::value<int>(&start)->default_value(0),
         "start at scan <arg> (i.e., neglects the first <arg> scans) "
         "[ATTENTION: counting naturally starts with 0]")
        ("end,e", po::value<int>(&end)->default_value(-1),
         "end after scan <arg>")
        ("format,f", po::value<IOType>(&iotype)->default_value(UOS),
         "using shared library <arg> for input. (chose F from {uos, uos_map, "
         "uos_rgb, uos_frames, uos_map_frames, old, rts, rts_map, ifp, "
         "riegl_txt, riegl_rgb, riegl_bin, zahn, ply})")
        ("max,M", po::value<int>(&maxDist)->default_value(-1),
         "neglegt all data points with a distance larger than <arg> 'units")
        ("min,m", po::value<int>(&minDist)->default_value(-1),
         "neglegt all data points with a distance smaller than <arg> 'units")
        ("scanserver,S", po::value<bool>(&scanserver)->default_value(false),
         "Use the scanserver as an input method and handling of scan data");

    po::options_description image("Panorama image options");
    image.add_options()
        ("range,r", "create range image")
        ("intensity,i", "create intensity image")
        ("width,w", po::value<int>(&width)->default_value(1280),
         "width of panorama")
        ("height,h", po::value<int>(&height)->default_value(960),
         "height of panorama")
        ("panorama,p", po::value<fbr::projection_method>(&ptype)->
         default_value(fbr::EQUIRECTANGULAR), "panorama type (EQUIRECTANGULAR, "
         "CYLINDRICAL, MERCATOR, RECTILINEAR, PANNINI, STEREOGRAPHIC, ZAXIS, "
         "CONIC)")
        ("num-images,N", po::value<int>(&nImages)->default_value(1),
         "number of images used for some projections")
        ("proj-param,P", po::value<int>(&pParam)->default_value(0),
         "special projection parameter")
        ("dump-pano,d", po::bool_switch(&dump_pano),
         "output panorama (useful to create marker image for watershed)");

    po::options_description range_image("Range image options");
    range_image.add_options()
        ("logarithm,L", po::bool_switch(&logarithm),
         "use the logarithm for range image panoramas")
        ("cutoff,C", po::value<float>(&cutoff)->default_value(0.0), // FIXME: percentage is the wrong word
         "percentage of furthest away data points to cut off to improve "
         "precision for closer values (values from 0.0 to 1.0)");

    po::options_description segment("Segmentation options");
    segment.add_options()
        ("segment,g", po::value<segment_method>(&stype)->
         default_value(PYR_MEAN_SHIFT), "segmentation method (THRESHOLD, "
         "ADAPTIVE_THRESHOLD, PYR_MEAN_SHIFT, PYR_SEGMENTATION, WATERSHED)")
        ("marker,K", po::value<string>(&marker),
         "marker mask for watershed segmentation")
        ("thresh,T", po::value<double>(&thresh),
         "threshold for threshold segmentation")
        ("maxlevel,X", po::value<int>(&maxlevel),
         "maximum level for meanshift segmentation")
        ("radius,R", po::value<int>(&radius),
         "radius for meanshift segmentation")
        ("links,l", po::value<double>(&pyrlinks),
         "error threshold for establishing the links for pyramid segmentation")
        ("clustering,c", po::value<double>(&pyrcluster),
         "error threshold for the segments clustering for pyramid "
         "segmentation")
        ("levels,E", po::value<int>(&pyrlevels)->default_value(4),
         "levels of pyramid segmentation")
        ("dump-seg,D", po::bool_switch(&dump_seg),
         "output segmentation image (for debugging)");

    po::options_description hidden("Hidden options");
    hidden.add_options()
        ("input-dir", po::value<string>(&dir), "input dir");

    // all options
    po::options_description all;
    all.add(generic).add(input).add(image).add(range_image).add(segment).add(hidden);

    // options visible with --help
    po::options_description cmdline_options;
    cmdline_options.add(generic).add(input).add(image).add(range_image).add(segment);

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
             << "bin/scan2segments -s 0 -e 0 -f riegl_txt --segment PYR_SEGMENTATION -l 50 -c 50 -E 4 -D -i ~/path/to/bremen_city\n" 
             << "bin/scan2segments -s 0 -e 0 -f riegl_txt --segment PYR_SEGMENTATION -l 255 -c 30 -E 2 -D -i ~/path/to/bremen_city\n";
        exit(0);
    }

    // option conflicts and dependencies
    conflicting_options(vm, "range", "intensity");
    option_dependency(vm, "logarithm", "range");
    option_dependency(vm, "cutoff", "range");

    // decide between range and intensity panorama
    if (vm.count("range"))
        itype = M_RANGE;
    else
        itype = M_INTENSITY;

    segmentation_option_dependency(vm, WATERSHED, "marker");
    segmentation_option_conflict(vm, WATERSHED, "thresh");
    segmentation_option_conflict(vm, WATERSHED, "maxlevel");
    segmentation_option_conflict(vm, WATERSHED, "radius");
    segmentation_option_conflict(vm, WATERSHED, "links");
    segmentation_option_conflict(vm, WATERSHED, "clustering");
    segmentation_option_conflict(vm, WATERSHED, "levels");

    segmentation_option_conflict(vm, THRESHOLD, "marker");
    segmentation_option_dependency(vm, THRESHOLD, "thresh");
    segmentation_option_conflict(vm, THRESHOLD, "maxlevel");
    segmentation_option_conflict(vm, THRESHOLD, "radius");
    segmentation_option_conflict(vm, THRESHOLD, "links");
    segmentation_option_conflict(vm, THRESHOLD, "clustering");
    segmentation_option_conflict(vm, THRESHOLD, "levels");

    segmentation_option_conflict(vm, PYR_MEAN_SHIFT, "marker");
    segmentation_option_conflict(vm, PYR_MEAN_SHIFT, "thresh");
    segmentation_option_dependency(vm, PYR_MEAN_SHIFT, "maxlevel");
    segmentation_option_dependency(vm, PYR_MEAN_SHIFT, "radius");
    segmentation_option_conflict(vm, PYR_MEAN_SHIFT, "links");
    segmentation_option_conflict(vm, PYR_MEAN_SHIFT, "clustering");
    segmentation_option_conflict(vm, PYR_MEAN_SHIFT, "levels");

    segmentation_option_conflict(vm, PYR_SEGMENTATION, "marker");
    segmentation_option_conflict(vm, PYR_SEGMENTATION, "thresh");
    segmentation_option_conflict(vm, PYR_SEGMENTATION, "maxlevel");
    segmentation_option_conflict(vm, PYR_SEGMENTATION, "radius");
    segmentation_option_dependency(vm, PYR_SEGMENTATION, "links");
    segmentation_option_dependency(vm, PYR_SEGMENTATION, "clustering");

    // correct pParam and nImages for certain panorama types
    if (ptype == fbr::PANNINI && pParam == 0) {
        pParam = 1;
        if(nImages < 3) nImages = 3;
    }
    if (ptype == fbr::STEREOGRAPHIC && pParam == 0) {
        pParam = 2;
        if(nImages < 3) nImages = 3;
    }
    if (ptype == fbr::RECTILINEAR && nImages < 3) {
        nImages = 3;
    }

    // add trailing slash to directory if not present yet
    if (dir[dir.length()-1] != '/') dir = dir + "/";
}

/*
 * retrieve a cv::Mat with x,y,z,r from a scan object
 * functionality borrowed from scan_cv::convertScanToMat but this function
 * does not allow a scanserver to be used, prints to stdout and can only
 * handle a single scan
 */
cv::Mat scan2mat(Scan *source)
{
    DataXYZ xyz = source->get("xyz");
    DataReflectance xyz_reflectance = source->get("reflectance");
    unsigned int nPoints = xyz.size();
    cv::Mat scan(nPoints,1,CV_32FC(4));
    scan = cv::Scalar::all(0);
    cv::MatIterator_<cv::Vec4f> it;
    it = scan.begin<cv::Vec4f>();
    for(unsigned int i = 0; i < nPoints; i++){
        float x, y, z, reflectance;
        x = xyz[i][0];
        y = xyz[i][1];
        z = xyz[i][2];
        reflectance = xyz_reflectance[i];

        //normalize the reflectance
        reflectance += 32;
        reflectance /= 64;
        reflectance -= 0.2;
        reflectance /= 0.3;
        if (reflectance < 0) reflectance = 0;
        if (reflectance > 1) reflectance = 1;

        (*it)[0] = x;
        (*it)[1] = y;
        (*it)[2] = z;
        (*it)[3] = reflectance;

        ++it;
    }
    return scan;
}

/*
 * convert a matrix of float values (range image) to a matrix of unsigned
 * eight bit characters using different techniques
 */
cv::Mat float2uchar(cv::Mat &source, bool logarithm, float cutoff)
{
    cv::Mat result(source.size(), CV_8U, cv::Scalar::all(0));
    float max = 0;
    // find maximum value
    if (cutoff == 0.0) {
        // without cutoff, just iterate through all values to find the largest
        for (cv::MatIterator_<float> it = source.begin<float>();
                it != source.end<float>(); ++it) {
            float val = *it;
            if (val > max) {
                max = val;
            }
        }
    } else {
        // when a cutoff is specified, sort all the points by value and then
        // specify the max so that <cutoff> values are larger than it
        vector<float> sorted(source.cols*source.rows);
        int i = 0;
        for (cv::MatIterator_<float> it = source.begin<float>();
                it != source.end<float>(); ++it, ++i) {
            sorted[i] = *it;
        }
        std::sort(sorted.begin(), sorted.end());
        max = sorted[(int)(source.cols*source.rows*(1.0-cutoff))];
        cout << "A cutoff of " << cutoff << " resulted in a max value of " << max << endl;
    }

    cv::MatIterator_<float> src = source.begin<float>();
    cv::MatIterator_<uchar> dst = result.begin<uchar>();
    cv::MatIterator_<float> end = source.end<float>();
    if (logarithm) {
        // stretch values from 0 to max logarithmically over 0 to 255
        // using the logarithm allows to represent smaller values with more
        // precision and larger values with less
        max = log(max+1);
        for (; src != end; ++src, ++dst) {
            float val = (log(*src+1)*255.0)/max;
            if (val > 255)
                *dst = 255;
            else
                *dst = (uchar)val;
        }
    } else {
        // stretch values from 0 to max linearly over 0 to 255
        for (; src != end; ++src, ++dst) {
            float val = (*src*255.0)/max;
            if (val > 255)
                *dst = 255;
            else
                *dst = (uchar)val;
        }
    }
    return result;
}

/*
 * from grayscale image, create a binary image using a fixed threshold
 */
cv::Mat calculateThreshold(vector<vector<cv::Vec3f>> &segmented_points,
        cv::Mat &img, vector<vector<vector<cv::Vec3f> > > extendedMap,
        double thresh)
{
    int i, j, idx;
    cv::Mat res;
    cv::threshold(img, res, thresh, 255, cv::THRESH_BINARY);
    segmented_points.resize(2);

    for (i = 0; i < res.rows; i++) {
        for (j = 0; j < res.cols; j++) {
            idx = res.at<uchar>(i,j);
            if (idx != 0)
                idx = 1;
            segmented_points[idx].insert(segmented_points[idx].end(),
								 extendedMap[i][j].begin(),
								 extendedMap[i][j].end());
        }
    }

    return res;
}

/*
 * calculate the pyramid mean shift segmentation of the image
 */
cv::Mat calculatePyrMeanShift(vector<vector<cv::Vec3f>> &segmented_points,
        cv::Mat &img, vector<vector<vector<cv::Vec3f> > > extendedMap,
        int maxlevel, int radius)
{
    int i, j, idx;
    cv::Mat imgGray, res, tmp;
    cvtColor(img, imgGray, CV_GRAY2BGR);
    cv::pyrMeanShiftFiltering(imgGray, tmp, radius, radius, maxlevel);
    cvtColor(tmp, res, CV_BGR2GRAY);

    // some colors will be empty
    // fill histogram first and then pick those entries that are not empty
    vector<vector<cv::Vec3f>> histogram(256);

    for (i = 0; i < res.rows; i++) {
        for (j = 0; j < res.cols; j++) {
            idx = res.at<uchar>(i,j);
            histogram[idx].insert(histogram[idx].end(),
						    extendedMap[i][j].begin(),
						    extendedMap[i][j].end());
        }
    }

    for (i = 0; i < 256; i++) {
        if (!histogram[i].empty()) {
            segmented_points.push_back(histogram[i]);
        }
    }

    return res;
}

///TODO: need to pass *two* thresh params, see: http://bit.ly/WmFeub
cv::Mat calculatePyrSegmentation(vector<vector<cv::Vec3f>> &segmented_points,
        cv::Mat &img, vector<vector<vector<cv::Vec3f> > > extendedMap,
        double thresh1, double thresh2, int pyrlevels)
{
    int i, j, idx;
    int block_size = 1000;
    IplImage ipl_img = img;
    IplImage* ipl_original = &ipl_img;
    IplImage* ipl_segmented = 0;

    CvMemStorage* storage = cvCreateMemStorage(block_size);
    CvSeq* comp = NULL;

    // the following lines are required because the level must not be more
    // than log2(min(width, height))
    ipl_original->width &= -(1<<pyrlevels);
    ipl_original->height &= -(1<<pyrlevels);

    ipl_segmented = cvCloneImage( ipl_original );

    // apply the pyramid segmentation algorithm
	/*
    cvPyrSegmentation(ipl_original, ipl_segmented, storage, &comp, pyrlevels, thresh1+1, thresh2+1); 
	*/
    // mapping of color value to component id
    map<int, int> mapping;
    unsigned int segments = comp->total;
    for (unsigned int cur_seg = 0; cur_seg < segments; ++cur_seg) {
      CvConnectedComp* cc = (CvConnectedComp*) cvGetSeqElem(comp, cur_seg);
      // since images are single-channel grayscale, only the first value is
      // of interest
      mapping.insert(pair<int, int>(cc->value.val[0], cur_seg));
    }

    segmented_points.resize(segments);

    uchar *data = (uchar *)ipl_segmented->imageData;
    int step = ipl_segmented->widthStep;
    for (i = 0; i < ipl_segmented->height; i++) {
        for (j = 0; j < ipl_segmented->width; j++) {
            idx = mapping[data[i*step+j]];
            segmented_points[idx].insert(segmented_points[idx].end(),
								  extendedMap[i][j].begin(),
								  extendedMap[i][j].end());
        }
    }

    // clearing memory
    cvReleaseMemStorage(&storage);

    cv::Mat res(ipl_segmented);
    return res;
}

/*
 * calculate the adaptive threshold
 */
cv::Mat calculateAdaptiveThreshold(vector<vector<cv::Vec3f>> &segmented_points,
        cv::Mat &img, vector<vector<vector<cv::Vec3f> > > extendedMap)
{
    int i, j, idx;
    cv::Mat res;
    cv::adaptiveThreshold(img, res, 255, CV_ADAPTIVE_THRESH_MEAN_C, CV_THRESH_BINARY, 49, 5);
    segmented_points.resize(2);

    for (i = 0; i < res.rows; i++) {
        for (j = 0; j < res.cols; j++) {
            idx = res.at<uchar>(i,j);
            if (idx != 0)
                idx = 1;
            segmented_points[idx].insert(segmented_points[idx].end(),
								 extendedMap[i][j].begin(),
								 extendedMap[i][j].end());
        }
    }

    return res;
}

/*
 * using a marker image, calculate the watershed segmentation
 * a marker image can be created from the panorama retrieved by using the
 * --dump-pano option
 */
cv::Mat calculateWatershed(vector<vector<cv::Vec3f>> &segmented_points,
        string &marker, cv::Mat &img, vector<vector<vector<cv::Vec3f> > > extendedMap)
{
    int i, j, idx;
    cv::Mat markerMask = cv::imread(marker, 0);
    vector<vector<cv::Point> > contours;
    vector<cv::Vec4i> hierarchy;
    cv::findContours(markerMask, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
    if (contours.empty())
        throw std::runtime_error("empty marker mask");
    cv::Mat markers(markerMask.size(), CV_32S);
    markers = cv::Scalar::all(0);
    int compCount = 0;
    for (int idx = 0; idx >= 0; idx = hierarchy[idx][0], compCount++ )
        cv::drawContours(markers, contours, idx,
					cv::Scalar::all(compCount+1), -1, 8, hierarchy, INT_MAX);
    if (compCount == 0)
        throw std::runtime_error("no component found");
    cv::Mat imgGray;
    cvtColor(img, imgGray, CV_GRAY2BGR);
    cv::watershed(imgGray, markers);

    segmented_points.resize(compCount);

    for (i = 0; i < markers.rows; i++) {
        for (j = 0; j < markers.cols; j++) {
            idx = markers.at<int>(i,j);
            if (idx > 0 && idx <= compCount) {
                segmented_points[idx-1].insert(segmented_points[idx-1].end(),
									  extendedMap[i][j].begin(),
									  extendedMap[i][j].end());
            }
        }
    }

    return markers;
}

/*
 * given a vector of segmented 3d points, write them out as uos files
 */
void write3dfiles(vector<vector<cv::Vec3f>> &segmented_points, string &segdir)
{
    unsigned int i;

    vector<ofstream*> outfiles(segmented_points.size());
    for (i = 0; i < segmented_points.size(); i++) {
        std::stringstream outfilename;
        outfilename << segdir << "/scan" << std::setw(3) << std::setfill('0') << i << ".3d";
        outfiles[i] = new ofstream(outfilename.str());
    }

    for (i = 0; i < segmented_points.size(); i++) {
        for (vector<cv::Vec3f>::iterator it=segmented_points[i].begin() ;
		   it < segmented_points[i].end();
		   it++) {
            (*(outfiles[i])) << (*it)[0] << " " << (*it)[1] << " " << (*it)[2] << endl;
        }
    }

    for (i = 0; i < segmented_points.size(); i++) {
        outfiles[i]->close();
    }
}

// write .pose files
// .frames files can later be generated from them using ./bin/pose2frames
void writeposefiles(int num, string &segdir, const double* rPos, const double* rPosTheta)
{
    for (int i = 0; i < num; i++) {
        std::stringstream posefilename;
        posefilename << segdir << "/scan" << std::setw(3) << std::setfill('0') << i << ".pose";
        ofstream posefile(posefilename.str());
        posefile << rPos[0] << " " << rPos[1] << " " << rPos[2] << endl;
        posefile << deg(rPosTheta[0]) << " "
			  << deg(rPosTheta[1]) << " "
			  << deg(rPosTheta[2]) << endl;
        posefile.close();
    }
}

void createdirectory(string segdir)
{
	int success = mkdir(segdir.c_str(), S_IRWXU|S_IRWXG|S_IRWXO);
    if (success == 0 || errno == EEXIST) {
        cout << "Writing segmentations to " << segdir << endl;
    } else {
        cerr << "Creating directory " << segdir << " failed" << endl;
        exit(1);
    }
}

int main(int argc, char **argv)
{
    // commandline arguments
    int start, end;
    bool scanserver;
    image_type itype;
    int width, height;
    int maxDist, minDist;
    int nImages, pParam;
    fbr::projection_method ptype;
    bool logarithm;
    float cutoff;
    string dir;
    IOType iotype;
    segment_method stype;
    string marker;
    bool dump_pano, dump_seg;
    double thresh;
    int maxlevel, radius;
    double pyrlinks, pyrcluster;
    int pyrlevels;

    parse_options(argc, argv, start, end, scanserver, itype, width, height,
            ptype, dir, iotype, maxDist, minDist, nImages, pParam, logarithm,
            cutoff, stype, marker, dump_pano, dump_seg, thresh, maxlevel,
            radius, pyrlinks, pyrcluster, pyrlevels);

    Scan::openDirectory(scanserver, dir, iotype, start, end);

    if(Scan::allScans.size() == 0) {
      cerr << "No scans found. Did you use the correct format?" << endl;
      exit(-1);
    }

    cv::Mat img, res;
    string segdir;

    for(ScanVector::iterator it = Scan::allScans.begin(); it != Scan::allScans.end(); ++it) {
        Scan* scan = *it;

        // apply optional filtering
        scan->setRangeFilter(maxDist, minDist);

        // create target directory
        segdir = dir + "segmented" + scan->getIdentifier();
        createdirectory(segdir);

        // create panorama
        fbr::panorama fPanorama(width, height, ptype, nImages, pParam, fbr::EXTENDED);
        fPanorama.createPanorama(scan2mat(scan));

        if (itype == M_RANGE) {
            // the range image has to be converted from float to uchar
            img = fPanorama.getRangeImage();
            img = float2uchar(img, logarithm, cutoff);
        } else {
            // intensity image
            img = fPanorama.getReflectanceImage();
        }

        // output panorama image
        if (dump_pano)
            imwrite(segdir+"/panorama.png", img);

        // will store the result of the segmentation
        vector<vector<cv::Vec3f>> segmented_points;

        if (stype == THRESHOLD) {
            res = calculateThreshold(segmented_points, img, fPanorama.getExtendedMap(), thresh);
        } else if (stype == PYR_MEAN_SHIFT) {
            res = calculatePyrMeanShift(segmented_points, img, fPanorama.getExtendedMap(),
								maxlevel, radius);
        } else if (stype == PYR_SEGMENTATION) {
            res = calculatePyrSegmentation(segmented_points, img, fPanorama.getExtendedMap(), pyrlinks, pyrcluster, pyrlevels);
        } else if (stype == ADAPTIVE_THRESHOLD) {
            res = calculateAdaptiveThreshold(segmented_points, img, fPanorama.getExtendedMap());
        } else if (stype == WATERSHED) {
            res = calculateWatershed(segmented_points, marker, img, fPanorama.getExtendedMap());
        }

        // output segmentation image
        if (dump_seg)
            imwrite(segdir+"/segmentation.png", res);

        // write .3d and .pose files
        write3dfiles(segmented_points, segdir);
        writeposefiles(segmented_points.size(), segdir, scan->get_rPos(), scan->get_rPosTheta());
    }
}
