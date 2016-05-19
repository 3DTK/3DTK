#include "b3dpsreader.h"
#include <iomanip>

bool B3DPSReader::mydeserialize(double* &points, uint32_t &nrpoints, uint64_t &linenr) {
    // read points
    if (!iffile3d.good()) return false;
    iffile3d.read((char*)&linenr, sizeof(uint64_t));
    if (!iffile3d.good()) return false;
    
    iffile3d.read((char*)&nrpoints, sizeof(uint32_t));
    
    uint32_t nrbytes;
    iffile3d.read((char*)&nrbytes, sizeof(uint32_t));
    
    points = new double[nrpoints * pointdim];
    iffile3d.read((char*)points, nrbytes-2);
    
    if (!readEOL(iffile3d) ) {
        iffile3d.close();
        return false;     
    }
    return true;
}

B3DPSReader::B3DPSReader(std::string _filename3d, std::string _filenameps) { 
    filename3d = _filename3d;
    filenameps = _filenameps;
    
    //    initPointType(_reflectance, _amplitude, _intensity, _rgb, _temperature, _type);
    // TODO encapsulate the meta point info stuff into its own class
    
    iffile3d.open ((filename3d).c_str(), std::ios::in | std::ios::binary);
    iffileps.open ((filenameps).c_str(), std::ios::in | std::ios::binary);
    
    minpose = UINT64_MAX;
    maxpose = 0;
    readTrajectory(filenameps, poses, minpose, maxpose);

    if (!deserializeHeader() ) {
        cerr << "Error in B3DPSReader! Could not deserialize B3D header." << endl;
    }
//    for (int i=0; i < 16; i++) { calibration[i] = 0.0;} 
    M4identity(calibration);

    // TODO
}


B3DPSReader::~B3DPSReader() {
    if (iffile3d.is_open()) {
        iffile3d.close();
    }
    if (iffileps.is_open()) {
        iffileps.close();
    }

    // TODO delete poses
    // TODO delete original points
}


void B3DPSReader::readTrajectory(string filename, map<uint64_t, double*> &poses, uint64_t &minpose, uint64_t &maxpose) {
    ifstream iffileps;
    iffileps.open ((filename).c_str(), std::ios::in | std::ios::binary);
    char buffer[1024];
    if (!iffileps.good()) return;
    
    iffileps.read(buffer, 3);
    if ( buffer[0] != 'M' || buffer[1] != 'P' || buffer[2] != 'S') {
        cerr << "ERROR: " << filename << " is not a MIM BPS file!" << endl;
        iffileps.close();
        return ;
    } 
    if (!readEOL(iffileps) ) {
        cerr << "ERROR: Read error in :" << filename << "." << endl;
        iffileps.close();
        return ;
    }
    
    // read poses
    while (iffileps.good()) {
        uint64_t linenr;
        iffileps.read((char*)&linenr, sizeof(uint64_t));
        if (!iffileps.good()) break; // read over end of file
        
        if (linenr < minpose) minpose = linenr;
        if (linenr > maxpose) maxpose = linenr;
        
        double *pose = new double[8];
        iffileps.read((char*)pose, sizeof(double)*8);

        // assume that values will not be greater than the size of the
        // observable universe in meter
        if (fabs(pose[0]) > 4.4e26
         || fabs(pose[1]) > 4.4e26
         || fabs(pose[2]) > 4.4e26
         || fabs(pose[3]) > 4.4e26
         || fabs(pose[4]) > 4.4e26
         || fabs(pose[5]) > 4.4e26
         || fabs(pose[6]) > 4.4e26
         || fabs(pose[7]) > 4.4e26) {
            std::cerr << "read implausibly large value" << std::endl;
            break;
        }

        poses[linenr] = pose;
       
//       cout << std::setprecision(20) << linenr << " " << pose[0] << " XYZ " << pose[1] << " " << pose[2] << " " << pose[3] << endl;
        if (!readEOL(iffileps) ) {
            iffileps.close();
            break;     
        }
    }
    
}   

bool B3DPSReader::deserializeHeader() {
    // read 3D scan
    if (!iffile3d.good()) return false; // no more files in the directory
    
    // read headers 
    char buffer[1024];
    
    iffile3d.read(buffer, 10);
    if ( buffer[0] != 'M' || buffer[1] != '3' || buffer[2] != 'D') {
        cerr << "ERROR: " << filename3d << " is not a MIM B3D file!" << endl;
        iffile3d.close();
        return false;
    } 
    bool _reflectance = buffer[4] != 0;
    bool _intensity   = buffer[5] != 0;
    bool _amplitude   = buffer[6] != 0;
    bool _temperature = buffer[7] != 0;
    bool _type        = buffer[8] != 0;
    bool _rgb         = buffer[9] != 0;
    initPointType(_reflectance, _amplitude, _intensity, _rgb, _temperature, _type);

    // TODO sanity check the number of bytes per point 
    int nrbytes;
    iffile3d.read((char*)&nrbytes, sizeof(int));
    
    if (!readEOL(iffile3d) ) {
        cerr << "ERROR: Read error in :" << filename3d << "." << endl;
        iffile3d.close();
        return false;
    }
    return true;
}


bool B3DPSReader::deserialize(vector<double*> &points, long &_linenr) {
    double *pts = 0;
    uint32_t nr_points;
    uint64_t linenr;
    bool result = mydeserialize(pts, nr_points, linenr);
    _linenr = linenr;
    
    if (!result) {
        delete[] pts;
        return false;
    }
   
    for (int i = 0; i < nr_points; i++) {
        points.push_back( &(pts[i*pointdim]) ); 
    }
    return true;
}
