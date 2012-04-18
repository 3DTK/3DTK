/**
 * @file
 * @brief Implementation of reading 3D scans
 * @author Andreas Nuechter. Jacobs University Bremen gGmbH
 * @author Jan Elseberg. Smart Systems Group, Jacobs University Bremen gGmbH, Germany. 
 */
/*
#include <libxml/tree.h>
#include <libxml/parser.h>
#include <libxml/xpath.h>
#include <libxml/xpathInternals.h>
*/
#include <libxml/tree.h>
#include <libxml/parser.h>
#include <libxml/xpath.h>
#include <libxml/xpathInternals.h>


#include "slam6d/scan_io_rxp.h"
#include "slam6d/scan_io_riegl_project.h"
#include "riegl/scanlib.hpp"
#include "slam6d/globals.icc"
#include <fstream>
using std::ifstream;
#include <iostream>
using std::cerr;
using std::endl;

#include <string.h>

#include <algorithm>
using std::swap;

#ifdef _MSC_VER
#include <windows.h>
#endif

using namespace scanlib;
using namespace std;
using namespace std::tr1;

static vector<string> filenames;
static vector<double *> eulers;

static xmlNodePtr srSeekChildNodeNamed(xmlNodePtr p, const char * name){
  if(p == NULL || name == NULL) return NULL;

  for(p=p->children; p!= NULL; p=p->next){
    if(p->name && (strcmp((char*)p->name,name) == 0)){
      return p;
    }
  }
  return NULL;
}

static void matrixToPos(const char* matrix, double *rPos, double *rPosTheta) {
    stringstream ss( stringstream::in | stringstream::out);
    ss << matrix;

    double inMatrix[16], tMatrix[16];
    for (int n=0; n<16; n++)
    {
      ss >> inMatrix[n];
    }
    tMatrix[0] = inMatrix[5];
    tMatrix[1] = -inMatrix[9];
    tMatrix[2] = -inMatrix[1];
    tMatrix[3] = -inMatrix[13];
    tMatrix[4] = -inMatrix[6];
    tMatrix[5] = inMatrix[10];
    tMatrix[6] = inMatrix[2];
    tMatrix[7] = inMatrix[14];
    tMatrix[8] = -inMatrix[4];
    tMatrix[9] = inMatrix[8];
    tMatrix[10] = inMatrix[0];
    tMatrix[11] = inMatrix[12];
    tMatrix[12] = -inMatrix[7];
    tMatrix[13] = inMatrix[11];
    tMatrix[14] = inMatrix[3];
    tMatrix[15] = inMatrix[15];

    Matrix4ToEuler(tMatrix, rPosTheta, rPos);
    rPos[0] *= 100;
    rPos[1] *= 100;
    rPos[2] *= 100;
}

static int getPaths(const char* filename) {
  double rPos[3];
  double rPosTheta[3];
  xmlDocPtr doc;
  xmlXPathContextPtr xpathCtx;
  xmlXPathObjectPtr xpathObj;
  const xmlChar* names = BAD_CAST "/project/scanpositions/scanposition";

  /* Load XML document */
  doc = xmlParseFile(filename);
  if (doc == NULL) {
    fprintf(stderr, "Error: unable to parse file \"%s\"\n", filename);
    return(-1);
  }

  /* Create xpath evaluation context */
  xpathCtx = xmlXPathNewContext(doc);
  if(xpathCtx == NULL) {
    fprintf(stderr,"Error: unable to create new XPath context\n");
    xmlFreeDoc(doc);
    return(-1);
  }

  /* Evaluate xpath expression */
  xpathObj = xmlXPathEvalExpression(names, xpathCtx);

  // parse the scanpositions
  xmlNodeSetPtr nodes = xpathObj->nodesetval;
  int size;
  int iter;

  size = (nodes) ? nodes->nodeNr : 0;

  for(iter = 0; iter < size; iter++) {
    xmlNodePtr position = nodes->nodeTab[iter];
    xmlNodePtr name = srSeekChildNodeNamed(position, "name");
    xmlNodePtr singlescans = srSeekChildNodeNamed(position, "singlescans");
    xmlNodePtr scan = srSeekChildNodeNamed(singlescans, "scan");
    xmlNodePtr file = srSeekChildNodeNamed(scan, "file");
    
    xmlNodePtr sop = srSeekChildNodeNamed(position, "sop");
    xmlNodePtr matrix = srSeekChildNodeNamed(sop, "matrix");


    char FILEN[1024];
    char POSITIONN[1024];
    char MATRIX[1024];

    if (file != 0 && name != 0 && matrix != 0) {
      sprintf(POSITIONN, "%s", xmlNodeGetContent(name ));
      sprintf(FILEN, "%s", xmlNodeGetContent(file ));
      sprintf(MATRIX, "%s", xmlNodeGetContent(matrix));
    
      // compute pose
      double *euler = new double[6];
      matrixToPos(MATRIX, rPos, rPosTheta);
      for (unsigned int i = 0; i < 3; i++) euler[i] = rPos[i];
      for (unsigned int i = 0; i < 3; i++) euler[i+3] = rPosTheta[i]; 
      eulers.push_back(euler);

      // put together filename
      string filename = "";
      filename = "SCANS/" + (string)POSITIONN + "/SINGLESCANS/" + (string)FILEN + ""; 
      //cout << rPos[0] << " " << rPos[1] << " " << rPos[2] << " " << filename << endl;
      cout << "Project Scan " << filename << " is mapped to index " << iter << endl;
      filenames.push_back(filename);
    }




  }



  /* Cleanup of XPath data */
  xmlXPathFreeObject(xpathObj);
  xmlXPathFreeContext(xpathCtx);

  /* free the document */
  xmlFreeDoc(doc);

  return(0);
}


/**
 * Reads specified scans from given directory in
 * the file format Riegl Laser Measurement GmbH 
 * uses. It will be compiled as shared lib.
 *
 * Scan poses will NOT be initialized after a call
 * to this function. Initial pose estimation works 
 * only with the -p switch, i.e., trusting the initial
 * estimations by Riegl.
 *
 * @param start Starts to read with this scan
 * @param end Stops with this scan
 * @param dir The directory from which to read
 * @param maxDist Reads only Points up to this Distance
 * @param minDist Reads only Points from this Distance
 * @param euler Initital pose estimates (will not be applied to the points
 * @param ptss Vector containing the read points
 */
int ScanIO_riegl_project::readScans(int start, int end, string &dir, int maxDist, int minDist,
						  double *euler, vector<Point> &ptss)
{
  static int fileCounter = start;

  // very first call to readScans...
  if (fileCounter == start) {
    // parse riegl projects xml file
    string xmlFileName = dir + "project.rsp";
  
    getPaths(xmlFileName.c_str());

  }
  string scanFileName;
  string poseFileName;

  if (end > -1 && fileCounter > end) return -1; // 'nuf read

  
  scanFileName = "file://" + dir + filenames[fileCounter];
    
  // read 3D scan

  cout << "Processing Scan " << scanFileName;
  cout.flush();



  for (unsigned int i = 0; i <= 5; i++) euler[i] = eulers[fileCounter][i]; 

  cout << " @ pose (" << euler[0] << "," << euler[1] << "," << euler[2]
	  << "," << deg(euler[3]) << "," << deg(euler[4]) << ","  << deg(euler[5]) << ")" << endl;
  
  // open scanfile
  std::tr1::shared_ptr<basic_rconnection> rc;
  rc = basic_rconnection::create(scanFileName);
  rc->open();

  // decoder splits the binary file into readable chunks
  decoder_rxpmarker dec(rc);
  // importer interprets the chunks
  importer imp(&ptss, maxDist, minDist);

  // iterate over chunks
  buffer  buf;
  for ( dec.get(buf); !dec.eoi(); dec.get(buf) ) {
    imp.dispatch(buf.begin(), buf.end());
  }

  //done
  rc->close();
  

  fileCounter++;
  
  return fileCounter-1;
}


/**
 * class factory for object construction
 *
 * @return Pointer to new object
 */
#ifdef _MSC_VER
extern "C" __declspec(dllexport) ScanIO* create()
#else
extern "C" ScanIO* create()
#endif
{
  return new ScanIO_riegl_project;
}


/**
 * class factory for object construction
 *
 * @return Pointer to new object
 */
#ifdef _MSC_VER
extern "C" __declspec(dllexport) void destroy(ScanIO *sio)
#else
extern "C" void destroy(ScanIO *sio)
#endif
{
  delete sio;
}

#ifdef _MSC_VER
BOOL APIENTRY DllMain(HANDLE hModule, DWORD dwReason, LPVOID lpReserved)
{
	return TRUE;
}
#endif
