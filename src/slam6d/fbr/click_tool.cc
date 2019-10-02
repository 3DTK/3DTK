/*
 * click_tool implementation
 *
 * Copyright (C) Jochen Barf
 *
 */

#include <stdio.h>
#include <fstream>
#include "slam6d/fbr/fbr_global.h"
#include "slam6d/fbr/scan_cv.h"
#include "slam6d/fbr/panorama.h"
#include "slam6d/fbr/feature.h"
#include "slam6d/fbr/feature_matcher.h"
#include "slam6d/fbr/registration.h"
#include "slam6d/fbr/feature_drawer.h"
#include "show/scancolormanager.h"
#include "show/show_Boctree.h"
#include "slam6d/point_type.h"
#include "show/show.h"
#include "slam6d/data_types.h"
#include "slam6d/Boctree.h"
#include "slam6d/basicScan.h"
#include "slam6d/fbr/click_tool.h"
#include <limits>
#include <boost/filesystem.hpp>

using namespace std;
using namespace fbr;

struct information{
    string scanInDir;
    string photoInDir;
    string  outDir;
    int start, end;
    IOType scanFormat;
    panorama_format photoFormat;
    int panoramaWidth, panoramaHeight;
    scanner_type scannerType;
    double minReflectance, maxReflectance;
    int minHorizAngle, maxHorizAngle;
    int minVertAngle, maxVertAngle;
    panorama_map_method mapMethod;
    projection_method projectionMethod;
    int numberOfImages;
    double projectionParam;
    bool panoramaSizeOptimization;
    float maxRange;
    float minRange;
    bool saveOct, loadOct;
    bool rotatePhoto;
    int photosPerScan;
    int photosPerPosition;
} info;

void usage(int argc, char** argv){
    printf("\n");
    printf("USAGE: %s scanInDir -s start -e end \n", argv[0]);
    printf("\n");
    printf("\n");
    printf("\tOptions:\n");
    printf("\t\t-f scanFormat\t\t\t input scan file format [RIEGL_TXT|RXP|ALL SLAM6D SCAN_IO]\n");
    printf("\t\t-h photoFormat\t\t\t input scan file format [PNG|JPEG|JPEG2000|TIFF]\n");
    printf("\t\t-W panoramaWidth\t\t panorama image width\n");
    printf("\t\t-H panoramaHeight\t\t panorama image height\n");
    printf("\t\t-t scannerType \t\t\t scanner type[NONE | RIEGL | FARO | MANUAL]\n");
    printf("\t\t-b minReflectance \t\t Min Reflectance for manual reflectance normalization\n");
    printf("\t\t-B maxReflectance \t\t Max Reflectance for manual reflectance normalization\n");
    printf("\t\t-m minHorizAngle \t\t Scanner horizontal view minAngle \n");
    printf("\t\t-w maxHorizAngle \t\t Scanner horizontal view maxAngle \n");
    printf("\t\t-n minVertAngle \t\t Scanner vertical view minAngle \n");
    printf("\t\t-x maxVertAngle \t\t Scanner vertical view maxAngle \n");
    printf("\t\t-d minRange \t\t\t Min range for manual range normalization \n");
    printf("\t\t-D maxRange \t\t\t Max range for manual range normalization \n");
    printf("\n");
    printf("\n");
    printf("\t\t-M mapMethod\t\t\t panorama map method [FARTHEST|EXTENDED|FULL]\n");
    printf("\t\t-p projectionMethod\t\t projection method [AZIMUTHAL|CONIC|CYLINDRICAL|EQUALAREACYLINDRICAL|EQUIRECTANGULAR|MERCATOR|PANNINI|RECTILINEAR|STEREOGRAPHIC|ZAXIS]\n");
    printf("\t\t-S panoramaFormatParam\t\t panorama format param panorama format related param mostly compression param\n");
    printf("\t\t-N numberOfImage\t\t number of Horizontal images used for some projections\n");
    printf("\t\t-P projectionParam\t\t special projection parameter (d for Pannini and r for stereographic)\n");
    printf("\t\t-a photosPerScan \t\t how many photos per scan were taken \n");
    printf("\t\t-c photosPerPosition \t\t how many photos per position were taken (HDR)\n");
    printf("\n");
    printf("\n");
    printf("\t\t-O outDir \t\t\t output directory if not stated same as input\n");
    printf("\t\t-r rotatePhoto \t\t\t rotates photo by 90 degrees counter clockwise\n");

    printf("\t\t-l loadOct \t\t\t load the Octtree\n");
    printf("\t\t-o saveOct \t\t\t save the Octtree\n");
    printf("\n");

    exit(1);
}

void parssArgs(int argc, char** argv, information& info){

    //default values
    info.scanFormat = RIEGL_TXT;
    info.photoFormat = JPEG;
    info.panoramaWidth = 3600;
    info.panoramaHeight = 1000;
    info.scannerType = NONE;
    info.minReflectance = -100;
    info.maxReflectance = 100;
    info.minHorizAngle = 0;
    info.maxHorizAngle = 360;
    info.minVertAngle = -40;
    info.maxVertAngle = 60;
    info.mapMethod = fbr::FARTHEST;
    info.projectionMethod = EQUIRECTANGULAR;
    info.numberOfImages = 1;
    //depend on the projection method
    info.projectionParam = 0;
    info.panoramaSizeOptimization = true;
    info.outDir = "";
    info.saveOct = false;
    info.loadOct = false;
    info.minRange = std::numeric_limits<float>::lowest();
    info.maxRange = std::numeric_limits<float>::max();
    info.rotatePhoto = false;
    info.photosPerScan = 1;
    info.photosPerPosition = 3;

    int c;
    opterr = 0;
    //reade the command line and get the options
    while ((c = getopt (argc, argv, "a:b:B:c:d:D:e:f:F:H:h:lm:M:n:N:oO:p:rs:t:w:W:x:")) != -1)
        switch (c){
            case 'a':
                info.photosPerScan = atoi(optarg);
                break;
            case 'b':
                info.minReflectance = atof(optarg);
                break;
            case 'B':
                info.maxReflectance = atof(optarg);
                break;
            case 'c':
                info.photosPerPosition = atoi(optarg);
                break;
            case 'e':
                info.end = atoi(optarg);
                break;
            case 'F':
                info.photoFormat = stringToPanoramaFormat(optarg);
                break;
            case 'D':
                info.maxRange = atof(optarg);
                break;
            case 'd':
                info.minRange = atof(optarg);
                break;
            case 'f':
                info.scanFormat = stringToScanFormat(optarg);
                break;
            case 'H':
                info.panoramaHeight = atoi(optarg);
                break;
            case 'h':
                info.photoFormat = stringToPanoramaFormat(optarg);
                break;
            case 'l':
                info.loadOct = true;
                break;
            case 'm':
                info.minHorizAngle = atoi(optarg);
                break;
            case 'M':
                info.mapMethod = stringToPanoramaMapMethod(optarg);
                break;
            case 'n':
                info.minVertAngle = atoi(optarg);
                break;
            case 'N':
                info.numberOfImages = atoi(optarg);
                break;
            case 'o':
                info.saveOct = true;
                break;
            case 'O':
                info.outDir = optarg;
                break;
            case 'p':
                info.projectionMethod = stringToProjectionMethod(optarg);
                break;
            case 's':
                info.start = atoi(optarg);
                break;
            case 'r':
                info.rotatePhoto = true;
                break;
            case 't':
                info.scannerType = stringToScannerType(optarg);
                break;
            case 'w':
                info.maxHorizAngle = atoi(optarg);
                break;
            case 'W':
                info.panoramaWidth = atoi(optarg);
                break;
            case 'x':
                info.maxVertAngle = atoi(optarg);
                break;
            case '?':
                cout<<"Unknown option character "<<optopt<<endl;
                usage(argc, argv);
                break;
            default:
                usage(argc, argv);
        }

    if(info.projectionMethod == PANNINI && info.projectionParam == 0){
        info.projectionParam = 1;
        if(info.numberOfImages < 2) info.numberOfImages = 2;
    }
    if(info.projectionMethod == STEREOGRAPHIC && info.projectionParam == 0){
        info.projectionParam = 2;
        if(info.numberOfImages < 2) info.numberOfImages = 2;
    }
    if(info.projectionMethod == RECTILINEAR && info.numberOfImages < 3)
        info.numberOfImages = 3;

    if (optind > argc - 1)
    {
        cout<<"Too few input arguments. At least scanInDir, start and stop are required."<<endl;
        usage(argc, argv);
    }

    info.scanInDir = argv[optind];
    if(info.photoInDir.empty()) info.photoInDir = info.scanInDir;
    if(info.outDir.empty()) info.outDir = info.scanInDir;
    if(info.outDir.back() != '/') info.outDir += '/';
}

void printInfo(information info){
    cout<<"Scan Input Dir= "<<info.scanInDir<<endl;
    cout<<"Photo Input Dir= "<<info.photoInDir<<endl;
    cout<<"Out Dir= "<<info.outDir<<endl;
    cout<<"Start= "<<info.start<<endl;
    cout<<"End= "<<info.end<<endl;
    cout<<"Scan Format= "<<scanFormatToString(info.scanFormat)<<endl;
    cout<<"Photo Format= "<<panoramaFormatToString(info.photoFormat)<<endl;
    cout<<"Width= "<<info.panoramaWidth<<endl;
    cout<<"Height= "<<info.panoramaHeight<<endl;
    cout<<"scannerType= "<<info.scannerType<<endl;
    cout<<"minReflectance= "<<info.minReflectance<<endl;
    cout<<"maxReflectance= "<<info.maxReflectance<<endl;
    cout<<"minHorizAngle= "<<info.minHorizAngle<<endl;
    cout<<"maxHorizAngle= "<<info.maxHorizAngle<<endl;
    cout<<"minVertAngle= "<<info.minVertAngle<<endl;
    cout<<"maxVertAngle= "<<info.maxVertAngle<<endl;
    cout<<"MinRange= "<<info.minRange<<endl;
    cout<<"MaxRange= "<<info.maxRange<<endl;
    cout<<"Map Method= "<<panoramaMapMethodToString(info.mapMethod)<<endl;
    cout<<"Projection Method= "<<projectionMethodToString(info.projectionMethod)<<endl;
    cout<<"photos per position= "<<info.photosPerPosition<<endl;
    cout<<"photos per scan= "<<info.photosPerScan<<endl;
    cout<<"Number of Images= "<<info.numberOfImages<<endl;
    cout<<"Projection Param= "<<info.projectionParam<<endl;
    cout<<"Panorama Size Optimization= "<<info.panoramaSizeOptimization<<endl;
    cout<<"Save Oct= "<<info.saveOct<<endl;
    cout<<"Load Oct= "<<info.loadOct<<endl;


    printf("\n\n\n");
    printf("Controls:\n\n");
    printf("+/-\t\tzoom\n");
    printf("t\t\ttoggle reflectance and range image\n");
    printf("left click\tadd point\n");
    printf("right click\tdelete point\n");
    printf("ESC\t\tfinish scan and write .ppr File\n");
    printf("left/right\tchange HDR photo\n");
    printf("up/down\t\tchange ppr file\n");
    printf("w/a/s/d\t\tup/left/down/right - naviagation\n");
    printf("f\t\tsee all point pairs\n");

    cout<<"\n-------------------------------"<<endl<<endl;
}

//------------------------  clicktool part------------------------------//

Mat inputs[2];                      //!< scan and photo images that are displayed
cv::Mat coordinateMap;              //!< the map that maps the pixels of the scan image to actual 3D coordinates
cv::Mat rangeImage;                 //!< the image of the scan that displays range information
cv::Mat reflectanceImage;           //!< the image of the scan that displays reflectance information
Image currentImage;                 //!< the scan image variance that is currently displayed, either range image or reflectance image

vector<PointPair> pointPairs;       //!< the collection of point pairs that the user selected in the current ppr file
vector<PointPair> allPointPairs;    //!< the collection of point pairs that the user selected in all ppr files in this folder
Point2i extraPoint[2];              //!< contains the point of a point pair that is already selected and waits for its partner to be selected
bool pointFound[2];                 //!< indicates for both images whether a point was selected
Scalar currentColor;                //!< the color of the point that was just selected and waits for its partner
MousePosition lastMousePosition;    //!< last known position of the mouse
float zoom[2];                      //!< indicates how much each image is zoomed
Point2i zoomCenter[2];              //!< defines the the center towards the image is zoomed
int windowHeight[2];                //!< holds the current heights of both windows
Point2i offset[2];                  //!< holds the current offset of the current view to the image origin of both images
int lastCreatedIndex;               //!< holds the index of the pointpair that was created last
int currentHDRIndex;                //!< holds the index of the hdr image the currently displayed
int currentPPRFileIndex;            //!< holds the index of the current ppr file
bool seeAll;						//!< specifies if all pointpairs from all ppr files are displayed
bool pprChanged;                    //!< holds true if a ppr was saved after allPointPairs was updated the last time


/**
 Rotates a point if the image was rotated before so that its on the same place

 @param _point   point that is to be rotated
 @param _picture the picture the point is from

 @return the rotated point
 */
Point2i getRotatedPoint(Point2i _point, Picture _picture);

/**
 Rotates an already rotated point back.

 @param _point   point that is to be unrotated
 @param _picture picture the point is from

 @return the unrotated point
 */
Point2i getUnrotatedPoint(Point2i _point, Picture _picture);

/**
 Toggles the enum between SCAN and PHOTO

 @param _picture the value that is to be toggled

 @return SCAN if _picture was PHOTO an vise versa
 */
Picture togglePicture(Picture _picture);

/**
 Calculates the factor by which the image _picture is scaled

 @param _picture the picture of which the scalefactor is of interest

 @return he factor by which the image _picture is scaled
 */
float scaleFactor(Picture _picture);

/**
 Generates a color by inserting random numbers

 @return a randomly generated color
 */
Scalar randomColor();

/**
 Gives the enum as string

 @param _picture the enum of which the string is needed

 @return the corresponding string
 */
string enum2String(Picture _picture);

/**
 Fetches the 3D coordinates of a image pixel and transforms it into a coordinate in the right handed coordinate system

 @param _point the pixe of ehich the 3D coordinate is of interest

 @return the 3D coordinates in right handed coordinate system
 */
Point3f getCoordinates(Point2i _point);

/**
 Draws and displays the image with the specified scale and zoom and draws all selected points into them

 @param _picture defines which image is to be painted
 */
void paint(Picture _picture);

/**
 Registers a selected point

 @param _point   the pixel that was selected
 @param _picture specifies in which image the point was selected
 */
void addPoint(Point2i _point, Picture _picture);

/**
 Unregisters a selected point

 @param _point   the pixel that is to be deleted
 @param _picture specifies the image the pixel is in
 */
void deletePoint(Point2i _point, Picture _picture);

/**
 Handles all actions of the mouse

 @param event    the exact action the mouse took
 @param x        the x coordinate of that action
 @param y        the y coordinate of that action
 @param flags    flags that further specify the event
 @param _picture specifies in which image the action was taken
 */
void mouseCallBack(int event, int x, int y, int flags, void* _picture);

/**
 zooms into the image that is currently in focus
 */
void zoomIn();

/**
 Zooms out of the image that is currently in focus
 */
void zoomOut();

/**
 Handles changes of the trackbars

 @param int      the value at which the indicator of the trackbar is located
 @param _picture defines the image that the trackbar corresponds to
 */
void trackBarCallback(int,void* _picture);

/**
 Toggles the current scan image between range and reflectance image
 */
void toggleImage();

/**
 Rotates image by 90 degrees counter clock wise if specified by user

 @param _input the image that is to be rotated

 @return the rotated image
 */
Mat rotateImage(Mat _input);

/**
 Reads the point pairs from a ppr file

 @param _scanNumber  the scan the ppr file corresponds to
 @param _photoNumber the photo the ppr file corresponds to

 @return a vector of all pointpairs in that file
 */
vector<PointPair> readPPRfile(int _scanNumber, int _photoNumber, int _pprFileIndex);

/**
 Writes the vector of pointpairs to a file using right handed coordinate system only

 @param _pointPairs  the point pairs that are to be saved
 @param _scanNumber  the scan the point pairs correspond to
 @param _photoNumber the photo the pointpairs correspond to
 */
void writePPRfile(vector<PointPair> _pointPairs, int _scanNumber, int _photoNumber);

/**
 Reads a photo from file

 @param _scanNumber  the scan the photo corresponds to
 @param _photoNumber the index of the photo within the scan

 @return the image read from file
 */
Mat readPhoto(int _scanNumber, int _photoNumber);


/**
 changes the displayed photo to the hdr photo with the next higher exposure time

 @param _scanNumber   the index of the current scan
 @param _currentPhoto the index of the current position
 */
void nextHDRPhoto(int _scanNumber, int & _currentPhoto);

/**
 changes the displayed photo to the hdr photo with the next lower exposure time

 @param _scanNumber   the index of the current scan
 @param _currentPhoto the index of the current position
 */
void nextHDRPhoto(int _scanNumber, int & _currentPhoto);

/**
 changes to the ppr file with the next higher index

 @param _pointPairs   the selceted point pairs of the current display
 @param _scanNumber   the index of the current scan
 @param _currentPhoto the index of the current position

 @return true if pattern was changed
 */
bool nextPattern(vector<PointPair> _pointPairs, int _scanNumber, int _photoNumber);

/**
 changes to the ppr file with the next lower index

 @param _pointPairs  the selceted point pairs of the current display
 @param _scanNumber   the index of the current scan
 @param _currentPhoto the index of the current position
 */
void lastPattern(vector<PointPair> _pointPairs, int _scanNumber, int _photoNumber);

/**
 reads all ppr files in the current folder

 @return all point pairs of all ppr files
 */
vector<PointPair> getAllPointPairs();

/**
 Reads a scan from file

 @param s          the index of the scan
 @param scanserver the scanserver
 */
void readScan(int s, bool & scanserver);



Point2i getRotatedPoint(Point2i _point, Picture _picture){

    if(_picture == PHOTO && info.rotatePhoto){
        return Point2i(_point.y, inputs[PHOTO].rows - _point.x);
    }

    return _point;
}

Point2i getUnrotatedPoint(Point2i _point, Picture _picture){

    if(_picture == PHOTO && info.rotatePhoto){
        return Point2i(inputs[PHOTO].rows - _point.y, _point.x);
    }

    return _point;
}

Picture togglePicture(Picture _picture){

    if (_picture == PHOTO) {
        return SCAN;
    } else {
        return PHOTO;
    }


}

float scaleFactor(Picture _picture){

    return ((float)windowHeight[_picture]/(float)inputs[_picture].rows)/zoom[_picture];

}


Scalar randomColor(){

    int r = rand() % 255;
    int g = rand() % 255;
    int b = rand() % 255;

    return Scalar(r,g,b);

}


string enum2String(Picture _picture){


    switch (_picture) {
        case SCAN:
            return "SCAN";
            break;

        case PHOTO:
            return "PHOTO";
            break;

        default:
            return "no title";
            break;
    }


}

Point3f getCoordinates(Point2i _point){
    Vec3f coordinates = coordinateMap.at<Vec3f>(_point);


    // coordinate map is in uos format z,-x,y (left handed coordinate system)
    Point3f pnt;
    pnt.x = coordinates[2];
    pnt.y = - coordinates[0];
    pnt.z = coordinates[1];

    return pnt;

}

void paint(Picture _picture){

    Mat output;

    float scale = scaleFactor(_picture);


    // resize the image with respect to scale due to window format and zoom
    resize(inputs[_picture], output,Size(0,0),scale,scale);


    // find sector of image that is to be displayed
    int height = windowHeight[_picture];
    int width = (inputs[_picture].cols * windowHeight[_picture]) / inputs[_picture].rows;
    int x = (float)zoomCenter[_picture].x*scale - (float)width/2.0;
    int y = (float)zoomCenter[_picture].y*scale - (float)height/2.0;


    // sector borders are out of image bounds then push sector back into image
    if (x+width >= output.cols){
        x = output.cols - width - 1;
    }

    if (y+height >= output.rows){
        y = output.rows - height - 1;
    }

    if (x < 0){
        x = 0;
    }

    if (y<0){
        y = 0;
    }

    // cropp image
    output = output(Rect(x,y,width,height));


    // update current offset to image origin
    offset[_picture].x = x/scale;
    offset[_picture].y = y/scale;


    // update center of zoom
    if(zoomCenter[_picture].x > inputs[_picture].cols - width/2/scale){
        zoomCenter[_picture].x = inputs[_picture].cols - width/2/scale;
    }

    if(zoomCenter[_picture].y > inputs[_picture].rows - height/2/scale){
        zoomCenter[_picture].y = inputs[_picture].rows - height/2/scale;
    }

    if(zoomCenter[_picture].x < width/2/scale){
        zoomCenter[_picture].x = width/2/scale;
    }

    if(zoomCenter[_picture].y < height/2/scale){
        zoomCenter[_picture].y = height/2/scale;
    }


    vector<PointPair> myPointPairs;


    if (seeAll){

    myPointPairs = getAllPointPairs();

	}


    myPointPairs.insert(myPointPairs.begin(),pointPairs.begin(),pointPairs.end());



    // draw all pointpairs
    for (unsigned int i = 0; i < myPointPairs.size(); i++) {

        Point2i point = (myPointPairs[i].points[_picture]-offset[_picture])*scale;

        circle(output,  point, 2, myPointPairs[i].color,2);

    }

    if (pointFound[_picture]){

        Point2i point = (extraPoint[_picture]-offset[_picture])*scale;

        circle(output, point, 2, currentColor,2);
    }

    imshow(enum2String(_picture), output);
}

void addPoint(Point2i _point, Picture _picture){

    //scale point back to image scale
    Point2i point = offset[_picture] +_point*(1.0/scaleFactor(_picture));

    // if a point already exists in the neighbourhood that do not add this point
    for (unsigned int i = 0; i < pointPairs.size(); i++) {
        Point2i dif = point - pointPairs[i].points[_picture];

        if ((dif.x*dif.x + dif.y*dif.y) <= 50.0){

            return;

        }

    }


    if(_picture == SCAN){


        // all pixels of the coordinateMap have a correspondence to a coordinate. if this is one of them then look in the close neighbourhood if there pixels with coordinate correspondence an take the closest
        Vec3f coordinates = coordinateMap.at<Vec3f>(point);
        if(coordinates[0] == 0 && coordinates[1] == 0 && coordinates[2] == 0){

            vector<Point2i> points;

            for (int x = point.x-3; x < point.x+3; ++x)
            {
                for (int y =point.y-3; y < point.y+3; ++y)
                {
                    if (y > 0 &&
                    	y < coordinateMap.rows &&
                    	x > 0 &&
                    	x < coordinateMap.cols)
                    {
                    coordinates = coordinateMap.at<Vec3f>(Point2i(x,y));

                    if(coordinates[0] != 0 || coordinates[1] != 0 || coordinates[2] != 0){

                        points.push_back(Point2i(x,y));


                    }
                    }
                }
            }

            Point2i nearest;
            float distance = std::numeric_limits<float>::max();
            for (unsigned int i = 0; i < points.size(); ++i)
            {
                Point2i diff = point - points[i];
                if(distance*distance > diff.x*diff.x + diff.y*diff.y){
                    nearest = points[i];
                    distance = diff.x*diff.x + diff.y*diff.y;
                }
            }

            if(points.size() > 0){

                cout << "alternative point " << nearest << " instead of "<<point <<" chosen!"<<endl;
                point = nearest;

            }else{
                cout << "no coordinate found at "<<point<<endl;
                return;
            }
        }
    }

    extraPoint[_picture] = point;
    pointFound[_picture] = true;

    // if this is the partner to an already selected point then save the point pair
    if(!pointFound[togglePicture(_picture)]){
        currentColor = randomColor();
    }else{

        PointPair pp;

        pp.points[PHOTO] = extraPoint[PHOTO];
        pp.points[SCAN] = extraPoint[SCAN];
        pp.coordinates = getCoordinates(extraPoint[SCAN]);
        pp.color = currentColor;

        lastCreatedIndex = pointPairs.size();

        pointPairs.push_back(pp);
        pointFound[PHOTO] = false;
        pointFound[SCAN] = false;


    }

    paint(_picture);

}

void deletePoint(Point2i _point, Picture _picture){

	int smallestIndex = -1;
    float smallestValue = std::numeric_limits<float>::max();
    // the selcetion with the cursor is not precise enaugh to select the exact pixel that is to be deleted, so search for a point in the neighbourhood and delete that
    for (int i = 0; i < (int) pointPairs.size(); i++) {

        Point2i dif = offset[_picture] + _point*(1.0/scaleFactor(_picture)) - pointPairs[i].points[_picture];

        if ((dif.x*dif.x + dif.y*dif.y) <= 100.0){

        	if ((dif.x*dif.x + dif.y*dif.y) < smallestValue){
        		smallestValue = (dif.x*dif.x + dif.y*dif.y);
        		smallestIndex = i;
        	}

        }

    }

    if(smallestIndex != -1){

    Point2i dif1 = offset[_picture]+_point*(1.0/scaleFactor(_picture)) - extraPoint[_picture];
    Point2i dif2 = offset[_picture] + _point*(1.0/scaleFactor(_picture)) - pointPairs[smallestIndex].points[_picture];
    float dist1 =(dif1.x*dif1.x + dif1.y*dif1.y);
    float dist2 =(dif2.x*dif2.x + dif2.y*dif2.y);

    if ( dist1 <= 100.0 && dist1  < dist2){

        pointFound[_picture] = false;
        paint(_picture);
    }else{

    if( smallestIndex == (int) pointPairs.size() -1 && !pointFound[SCAN] && !pointFound[PHOTO] && lastCreatedIndex == smallestIndex){

                PointPair current = pointPairs[smallestIndex];
                pointPairs.erase(pointPairs.begin()+smallestIndex);

                pointFound[_picture] = false;

                extraPoint[togglePicture(_picture)] = pointPairs[smallestIndex].points[togglePicture(_picture)];
                pointFound[togglePicture(_picture)] = true;
                currentColor = current.color;

            }else{

                pointPairs.erase(pointPairs.begin()+smallestIndex);

            }

            paint(SCAN);
            paint(PHOTO);
        }
    }else{

    	Point2i dif = offset[_picture]+_point*(1.0/scaleFactor(_picture)) - extraPoint[_picture];

    float dist =(dif.x*dif.x + dif.y*dif.y);


    if ( dist <= 100.0){

        pointFound[_picture] = false;
        paint(_picture);
    }

    }

}

void mouseCallBack(int event, int x, int y, int flags, void* _picture)
{
    Picture * picture = (Picture *) _picture;

    Point2i point= Point2i(x,y);

    if  ( event == EVENT_LBUTTONDOWN )
    {
        addPoint(point,*picture);


    }
    else if  ( event == EVENT_RBUTTONDOWN )
    {

        deletePoint(point,*picture);

    }else if(event == EVENT_MOUSEMOVE  ){
        lastMousePosition.position =  offset[*picture] + point*(1.0/scaleFactor(*picture));
        lastMousePosition.picture = *picture;
    }

}


void zoomIn(){

    zoom[lastMousePosition.picture] /= zoomIncrement;
    zoomCenter[lastMousePosition.picture] = lastMousePosition.position;
    paint(lastMousePosition.picture);

}
void zoomOut(){



    zoom[lastMousePosition.picture] *= zoomIncrement;
    if (zoom[lastMousePosition.picture] > 1.0) {

        zoom[lastMousePosition.picture] = 1.0;

    }
    paint(lastMousePosition.picture);



}


void trackBarCallback(int,void* _picture){

    Picture * picture = (Picture *) _picture;

    if ((windowHeight[*picture]*inputs[*picture].cols)/inputs[*picture].rows < minWindowWidth) {
        windowHeight[*picture] = (minWindowWidth*inputs[*picture].rows)/inputs[*picture].cols;
    }
    paint(*picture);

}

void toggleImage(){

    int imageInt = (int) currentImage + 1;

    if(imageInt > 1){
        imageInt = 0;
    }
    currentImage = (Image) imageInt;

    switch(currentImage){

        case Image::RANGE:
            inputs[SCAN] = rangeImage;
            break;

        case Image::REFLECTANCE:
            inputs[SCAN] = reflectanceImage;
            break;


    }
    paint(SCAN);

}

Mat rotateImage(Mat _input){

    // counter clock wise
    Mat output = _input.clone();
    if(info.rotatePhoto){
        transpose(output, output);
        flip(output, output,0);
    }
    return output;

}

vector<PointPair> getAllPointPairs(){
vector<PointPair> output;

	if(pprChanged){

		cout<< "reading all ppr files, this may take a while ... "<<flush;


	vector<string> paths;
	vector<string> extensions;
    extensions.push_back(".ppr");
    extensions.push_back(".PPR");


	 boost::filesystem::directory_iterator end_itr;
	        for (boost::filesystem::directory_iterator itr(info.outDir); itr != end_itr; ++itr)
	        {
	            if (is_regular_file(itr->path())) {
	                for (unsigned int i = 0; i < extensions.size(); i++) {
	                    if (itr->path().extension() == extensions[i]) {
	                        string current_file = itr->path().string();
	                        paths.push_back(current_file);
	                        break;
	                    }
	                }

	            }
	        }



	    for (unsigned int i = 0; i < paths.size(); ++i)
	    {
	    	if(boost::filesystem::exists(paths[i])){

        ifstream inputFile;
        inputFile.open(paths[i]);
        string buffer;

        while (getline(inputFile, buffer)) {

            stringstream sstm;
            sstm << buffer;
            string number;

            PointPair ppr;
            Point3f scanPoint;
            Point2i photoPoint;

            getline(sstm, number, ' ');
            photoPoint.x = stoi(number);
            getline(sstm, number, ' ');
            photoPoint.y = stoi(number);

            getline(sstm, number, ' ');
            scanPoint.x = stof(number);

            getline(sstm, number, ' ');
            scanPoint.y = stof(number);

            getline(sstm, number, ' ');
            scanPoint.z = stof(number);

            ppr.points[PHOTO] = getRotatedPoint(photoPoint, PHOTO);
            ppr.color = randomColor();

            //ppr files dont include the oixel value of the 3D coordinate in the panorama image, so search vor the 3D coordinate in the image and get the corresponding pixel

            Point2i nearest;
	    float distance = std::numeric_limits<float>::max();
            for(int y = 0; y < coordinateMap.rows; y++){
                for(int x = 0; x < coordinateMap.cols; x++){
                    Point3f coordinates = getCoordinates(Point2i(x,y));
 			Point3f diff = scanPoint - coordinates;
			float thisDistance = diff.x*diff.x + diff.y*diff.y + diff.z*diff.z;
			if(thisDistance < distance){
				nearest = Point2i(x,y);
				distance = thisDistance;
			}
                }
            }
		ppr.points[SCAN] = nearest;
                ppr.coordinates = scanPoint;
                output.push_back(ppr);
        }
    }

	    }

	    allPointPairs = output;
	    pprChanged = false;
	    cout << "done!"<<endl;
	}else{

		output = allPointPairs;

	}



return output;


}


vector<PointPair> readPPRfile(int _scanNumber, int _photoNumber, int _pprFileIndex){

    vector<PointPair> output;

    string filename = info.outDir+"photo"+to_string(_scanNumber*info.photosPerScan+_photoNumber, 3)+"_"+to_string(_pprFileIndex)+".ppr";
    cout << "reading " << filename;
    if(boost::filesystem::exists(filename)){

        ifstream inputFile;
        inputFile.open(filename);
        string buffer;

        while (getline(inputFile, buffer)) {

            stringstream sstm;
            sstm << buffer;
            string number;

            PointPair ppr;
            Point3f scanPoint;
            Point2i photoPoint;

            getline(sstm, number, ' ');
            photoPoint.x = stoi(number);
            getline(sstm, number, ' ');
            photoPoint.y = stoi(number);

            getline(sstm, number, ' ');
            scanPoint.x = stof(number);

            getline(sstm, number, ' ');
            scanPoint.y = stof(number);

            getline(sstm, number, ' ');
            scanPoint.z = stof(number);

            ppr.points[PHOTO] = getRotatedPoint(photoPoint, PHOTO);
            ppr.color = randomColor();

            //ppr files dont include the pixel value of the 3D coordinate in the panorama image, so search vor the 3D coordinate in the image and get the corresponding pixel

            Point2i nearest;
	    float distance = std::numeric_limits<float>::max();
            for(int y = 0; y < coordinateMap.rows; y++){
                for(int x = 0; x < coordinateMap.cols; x++){
                    Point3f coordinates = getCoordinates(Point2i(x,y));
 			Point3f diff = scanPoint - coordinates;
			float thisDistance = diff.x*diff.x + diff.y*diff.y + diff.z*diff.z;
			if(thisDistance < distance){
				nearest = Point2i(x,y);
				distance = thisDistance;
			}
                }
            }
		ppr.points[SCAN] = nearest;
                ppr.coordinates = scanPoint;
                output.push_back(ppr);
        }
    }else{
        cout << " ... it does not exist!";
    }
    cout << endl;

    return output;

}


void writePPRfile(vector<PointPair> _pointPairs, int _scanNumber, int _photoNumber){


    ofstream output;

    string filename = info.outDir+"photo"+to_string(_scanNumber*info.photosPerScan+_photoNumber, 3)+"_"+to_string(currentPPRFileIndex)+".ppr";
    cout<<"writing "<< filename;

    if(boost::filesystem::exists(filename)){

        cout<< " ... it exists and will be replaced!";

    }
    cout<<endl;

    output.open (filename);


    for (unsigned int i = 0; i < _pointPairs.size(); i++) {
        Point2i photoPoint = getUnrotatedPoint(_pointPairs[i].points[PHOTO], PHOTO);

        // 3D coordinates in right handed coordinate system !!!
        output      << std::setprecision(10)<< (float)photoPoint.x << " "
                    << (float)photoPoint.y << " "
                    << _pointPairs[i].coordinates.x << " "
                    << _pointPairs[i].coordinates.y << " "
                    << _pointPairs[i].coordinates.z << " "<<endl;
    }

    output.close();

    pprChanged = true;

}


Mat readPhoto(int _scanNumber, int _photoNumber){

    string filename = info.photoInDir+"photo"+to_string(_scanNumber*info.photosPerScan+_photoNumber, 3)+"_"+to_string(currentHDRIndex)+"."+panoramaFormatToFileFormatString(info.photoFormat);
    cout << "reading " << filename;

    Mat output;
    if(boost::filesystem::exists(filename)){
        output = rotateImage(imread(filename));
    }else{
        cout << " ... it does not exist!";
    }
    cout << endl;

    return output;
}

void nextHDRPhoto(int _scanNumber, int & _currentPhoto){

    currentHDRIndex++;
    if(currentHDRIndex >= info.photosPerPosition){
        currentHDRIndex = 0;
    }

    inputs[PHOTO]= readPhoto(_scanNumber, _currentPhoto);
    paint(PHOTO);

}
void lastHDRPhoto(int _scanNumber, int & _currentPhoto){

    currentHDRIndex--;
    if(currentHDRIndex < 0){
        currentHDRIndex = info.photosPerPosition-1;
    }


    inputs[PHOTO]= readPhoto(_scanNumber, _currentPhoto);
    paint(PHOTO);

}

bool nextPattern(vector<PointPair> _pointPairs, int _scanNumber, int _photoNumber){
seeAll = false;
  if(pointPairs.size() > 0){
  writePPRfile(pointPairs,_scanNumber,_photoNumber);

  pointPairs.clear();
  currentPPRFileIndex++;

   pointPairs = readPPRfile(_scanNumber, _photoNumber, currentPPRFileIndex);

  paint(PHOTO);
  paint(SCAN);
  return true;
}
return false;
  }
void lastPattern(vector<PointPair> _pointPairs, int _scanNumber, int _photoNumber){

	seeAll = false;
  if (currentPPRFileIndex > 0){
  writePPRfile(pointPairs,_scanNumber,_photoNumber);

  pointPairs.clear();
  currentPPRFileIndex--;



  pointPairs = readPPRfile(_scanNumber, _photoNumber, currentPPRFileIndex);


    paint(PHOTO);
    paint(SCAN);
  }

  }

void readScan(int s, bool & scanserver){


    scan_cv scan(info.scanInDir, s, info.scanFormat, scanserver, info.scannerType, info.loadOct, info.saveOct, true, true, -1, -1, info.minReflectance, info.maxReflectance);

    scan.convertScanToMat();


    //init the panorama
    fbr::panorama pImage;
    info.panoramaSizeOptimization = true;
    pImage.init(info.panoramaWidth, info.panoramaHeight, info.projectionMethod, info.numberOfImages, info.projectionParam, fbr::FARTHEST, scan.getZMin(), scan.getZMax(), info.minHorizAngle, info.maxHorizAngle, info.minVertAngle, info.maxVertAngle, info.panoramaSizeOptimization, true, true, true);

    //create panorama
    pImage.createPanorama(scan.getMatScan(), scan.getMatScanColor());
    //get the new panorama image size incase of optimized panorama size
    info.panoramaWidth = pImage.getImageWidth();
    info.panoramaHeight = pImage.getImageHeight();


    coordinateMap = pImage.getMap();
    rangeImage = pImage.getRangeImage();
    reflectanceImage = pImage.getReflectanceImage();


    //normalize range image

    float rangeMin = std::numeric_limits<float>::max();
    float rangeMax = std::numeric_limits<float>::min();


    for(int y = 0; y < rangeImage.rows; y++){
        for(int x = 0; x < rangeImage.cols; x++){

            float currentRange = rangeImage.at<float>(y,x);

            if(currentRange >= info.minRange && currentRange <= info.maxRange && fabs(currentRange) > 0.000001){

                if(currentRange < rangeMin){
                    rangeMin = currentRange;
                }
                if(currentRange > rangeMax){
                    rangeMax = currentRange;
                }
            }

        }
    }

    cv::Mat rangeTemp;
    rangeTemp.create(rangeImage.rows,rangeImage.cols,CV_8UC3);
    cv::Mat reflectanceTemp;
    reflectanceTemp.create(reflectanceImage.rows,reflectanceImage.cols,CV_8UC3);
    Vec3b zeros3b = Vec3b(0,0,0);
    //noramlize reflectance

    for(int y = 0; y < rangeTemp.rows; y++){
        for(int x = 0; x < rangeTemp.cols; x++){

            float currentRange = rangeImage.at<float>(y,x);
            unsigned char currentReflectance = reflectanceImage.at<unsigned char>(y,x);
            reflectanceTemp.at<Vec3b>(y,x) = Vec3b(currentReflectance,currentReflectance,currentReflectance);

            if( fabs(currentRange) > 0.000001 ){
                int value = (int)(255.0*((currentRange-rangeMin)/(rangeMax)));
                if(value > 255) value = 255;
                if(value < 0) value = 0;
                Vec3b value3b = Vec3b(value,value,value);
                rangeTemp.at<Vec3b>(y,x) = value3b;
            }else{
                rangeTemp.at<Vec3b>(y,x) = zeros3b;
            }

        }
    }

    rangeImage = rangeTemp;
    reflectanceImage = reflectanceTemp;
}

int main(int argc, char** argv){

 cout << "using opencv "<< CV_VERSION <<endl;
    if(strcmp(CV_VERSION, "2.4.9") != 0) {
        cout << "Please notice that this code was written for Opencv 2.4.9." << endl;
    }

    cout << "using boost " << BOOST_VERSION / 100000 << "." << BOOST_VERSION / 100 % 1000 << "." << BOOST_VERSION % 100 <<endl;
    if(BOOST_VERSION != 105500)cout << "Please notice that this code was written for boost 1.55.0."<<endl;

    Mat loading;
    loading = Mat::zeros(300,600,CV_8UC3);
    currentHDRIndex = 0;
    putText(loading,"loading ...", Point2i(100,150),CV_FONT_HERSHEY_SIMPLEX,2.0,Scalar(50,50,255),5);

    parssArgs(argc, argv, info);
    printInfo(info);

    bool scanserver = false;
    int currentPhoto;
    for(int s = info.start; s <= info.end; s++){



        readScan(s,scanserver);

        pointFound[PHOTO] = false;
        pointFound[SCAN] = false;
        zoom[SCAN] = 1.0;
        zoom[PHOTO] = 1.0;
        windowHeight[SCAN] = 768;
        windowHeight[PHOTO] = 768;
        Point2i temp = Point2i(0,0);
        offset[SCAN] = temp;
        offset[PHOTO] = temp;
        Picture scanEnum = SCAN;
        Picture photoEnum = PHOTO;
        lastCreatedIndex = -1;
        pointPairs.clear();
        currentPhoto = 0;
        currentPPRFileIndex = 0;
        seeAll = true;
        pprChanged = true;

        namedWindow(enum2String(SCAN),WINDOW_AUTOSIZE);
        namedWindow(enum2String(PHOTO),WINDOW_AUTOSIZE);



        setMouseCallback(enum2String(SCAN), mouseCallBack, (void*)&scanEnum);
        setMouseCallback(enum2String(PHOTO), mouseCallBack, (void*)&photoEnum);

        namedWindow("CONTROLS",WINDOW_NORMAL);
        resizeWindow("CONTROLS", 600, 40);


        createTrackbar("ScanSize", "CONTROLS", &windowHeight[SCAN], maxWindowHeight, &trackBarCallback,(void*)&scanEnum);
        createTrackbar("PhotoSize", "CONTROLS", &windowHeight[PHOTO], maxWindowHeight,&trackBarCallback,(void*)&photoEnum);

        inputs[SCAN] = rangeImage;
        currentImage = RANGE;

        inputs[PHOTO] = readPhoto(s,currentPhoto);
        pointPairs = readPPRfile(s,currentPhoto, currentPPRFileIndex);


        paint(SCAN);
        paint(PHOTO);



        int key = 0;
        char cKey =0;
        while(key != 1048603 && key != 27){
            key = (int) waitKey();
            cKey = (char) key;

            if(key ==  1048619 || key == 1114027 || cKey == '+'){
                zoomIn();
            }
            if(key ==  1048621 || key ==1114029 || cKey == '-'){
                zoomOut();
            }
            if(key == 1048692 || cKey == 't'){
                toggleImage();
            }

            if(cKey == 'w' || key == 1048695){

                zoomCenter[lastMousePosition.picture].y -= (int) (0.01*(float)inputs[lastMousePosition.picture].rows/scaleFactor(lastMousePosition.picture));
                paint(lastMousePosition.picture);
            }
            if(cKey == 'a' || key == 1048673){
                zoomCenter[lastMousePosition.picture].x -= (int) (0.01*(float)inputs[lastMousePosition.picture].cols/scaleFactor(lastMousePosition.picture));
                paint(lastMousePosition.picture);
            }
            if(cKey == 's' || key == 1048691){
                zoomCenter[lastMousePosition.picture].y += (int) (0.01*(float)inputs[lastMousePosition.picture].rows/scaleFactor(lastMousePosition.picture));
                paint(lastMousePosition.picture);
            }
            if(cKey == 'd' || key == 1048676){
                zoomCenter[lastMousePosition.picture].x += (int) (0.01*(float)inputs[lastMousePosition.picture].cols/scaleFactor(lastMousePosition.picture));
                paint(lastMousePosition.picture);
            }

            if(key == 1113939 || key == 65363){
                nextHDRPhoto(s, currentPhoto);


            }
            if(key == 1113937 || key == 65361){
                lastHDRPhoto(s, currentPhoto);
            }

            if(key == 1113938 || key == 65362){
                nextPattern(pointPairs,s,currentPhoto);
            }
            if(key == 1113940 || key == 65364){
                lastPattern(pointPairs,s,currentPhoto);
            }

            if(cKey == 'f'){
                if(seeAll){
                	seeAll=false;
                }else{
                	seeAll=true;
                }
                paint(PHOTO);
                paint(SCAN);
            }

        }

        writePPRfile(pointPairs,s,currentPhoto);

        imshow(enum2String(SCAN),loading);
        imshow(enum2String(PHOTO),loading);
        waitKey(100);
    }

}
