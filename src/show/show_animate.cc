#include "show/show_animate.h"

int nr_interpolations = 0;
int imageNr = 0;

void calcUpPath()
{

  // if camera list is empty then return
  if(cams.empty())	return;
  PointXY temp ;
  std::vector<PointXY> ups_listXY, ups_listXZ;

  // to interpolate with the xy coordinate.
  for (unsigned int i = 0; i < ups.size(); i++) {
    temp.x = ups[i].x;
    temp.y = ups[i].y;
    ups_listXY.push_back(temp);
  }


  // to interpolate with the xz coordinate.
  for (unsigned int i = 0; i < ups.size(); i++) {
    temp.x = ups[i].x;
    temp.y = ups[i].z;
    ups_listXZ.push_back(temp);
  }

  // now get the nurbs path for the individual xy and xz plane
  ups_vectorX = cam_nurbs_path.getNurbsPath(ups_listXY,
								    nr_interpolations,
								    inter_by_dist);
  ups_vectorZ = cam_nurbs_path.getNurbsPath(ups_listXZ,
								    nr_interpolations,
								    inter_by_dist);
}

void calcLookAtPath()
{

  // if camera list is empty then return
  if(cams.empty())	return;
  PointXY temp ;
  std::vector<PointXY> lookat_listXY, lookat_listXZ;

  // to interpolate with the xy coordinate.
  for (unsigned int i = 0; i < lookats.size(); i++) {
    temp.x = lookats[i].x;
    temp.y = lookats[i].y;
    lookat_listXY.push_back(temp);
  }

  // to interpolate with the xz coordinate.
  for (unsigned int i = 0; i < lookats.size(); i++) {
    temp.x = lookats[i].x;
    temp.y = lookats[i].z;
    lookat_listXZ.push_back(temp);
  }

  // now get the nurbs path for the individual xy and xz plane
  lookat_vectorX = cam_nurbs_path.getNurbsPath(lookat_listXY,
									  nr_interpolations,
									  inter_by_dist);
  lookat_vectorZ = cam_nurbs_path.getNurbsPath(lookat_listXZ,
									  nr_interpolations,
									  inter_by_dist);
}

void calcPath()
{
  if (cams.empty()) return;
  PointXY temp ;
  std::vector<PointXY> path_listXY, path_listXZ;

  // if camera list is empty then return
  // to interpolate with the xy coordinate.
  for(unsigned int i = 0;i < cams.size(); i++) {
    temp.x = cams[i].x;
    temp.y = cams[i].y;
    path_listXY.push_back(temp);
  }

  // to interpolate with the xz coordinate.
  for(unsigned int i=0;i<cams.size();i++){
    temp.x = cams[i].x;
    temp.y = cams[i].z;
    path_listXZ.push_back(temp);
  }

  nr_interpolations = calcNoOfPoints(path_listXY, path_listXZ);
  calcInterpolatedCameras(path_listXY, path_listXZ);
  // now get the nurbs path for the individual xy and xz plane
  path_vectorX = cam_nurbs_path.getNurbsPath(path_listXY,
									nr_interpolations,
									inter_by_dist);
  path_vectorZ = cam_nurbs_path.getNurbsPath(path_listXZ,
									nr_interpolations,
									inter_by_dist);
}

void updateCamera()
{
  updateCamControls();
  calcPath();
  calcLookAtPath();
  calcUpPath();
}


//-----------------------------------------------------------------------------

/**
 * This function saves the path drawn on the
 * screen to a file.
 */
void savePath(int dummy)
{
  // output file stream
  std::ofstream pathfile;

  // open the output file
  pathfile.open(path_file_name, std::ios::out);

  // if file not found then show error
  if(!pathfile){
    std::cerr << "Error creating the path file." << std::endl;
    return;
  }

  // store all the relevant information about the
  // individual cameras in this file. however, start
  // with the total number of cameras in the first line
  pathfile << cams.size() << std::endl;
  for(unsigned int i =0; i< cams.size();i++) {
    pathfile << cams[i].x << std::endl;
    pathfile << cams[i].y << std::endl;
    pathfile << cams[i].z << std::endl;
    pathfile << lookats[i].x << std::endl;
    pathfile << lookats[i].y << std::endl;
    pathfile << lookats[i].z << std::endl;
    pathfile << ups[i].x << std::endl;
    pathfile << ups[i].y << std::endl;
    pathfile << ups[i].z << std::endl;
  }

  // close the file after writing
  pathfile.clear();
  pathfile.close();
}


//----------------------------------------------------------------------------

/**
 * This function loads the path from the given
 * path file.
 */
void loadPath(int dummy) {

  // some temporary variables
  unsigned int length;
  float x, y, z;
  float lx, ly, lz;
  float ux, uy, uz;

  // buffer variable
  char buffer[2048];

  // file stream
  std::fstream pathFile;

  // check if other files are open or not
  if(pathFile.is_open()){
    // if open then close the file
    pathFile.clear();
    pathFile.close();
  }

  // clear the camlist first
  cams.clear();
  lookats.clear();
  ups.clear();

  // open the path file
  pathFile.open(path_file_name, std::ios::in);

  // read the first line containing info
  // about no of cameras in the file
  pathFile.getline(buffer,2048);
  length = atoi(buffer);

  // now for each camera extract the stored
  // information
  for(unsigned int i=0;i<length;i++) {
    pathFile.getline(buffer,2048);
    x = atof(buffer);
    pathFile.getline(buffer,2048);
    y = atof(buffer);
    pathFile.getline(buffer,2048);
    z = atof(buffer);

    pathFile.getline(buffer,2048);
    lx = atof(buffer);
    pathFile.getline(buffer,2048);
    ly = atof(buffer);
    pathFile.getline(buffer,2048);
    lz = atof(buffer);

    pathFile.getline(buffer,2048);
    ux = atof(buffer);
    pathFile.getline(buffer,2048);
    uy = atof(buffer);
    pathFile.getline(buffer,2048);
    uz = atof(buffer);

    // feed the information to create a new
    // camera with those values
    Point p(x,y,z);
    Point l(lx,ly,lz);
    Point u(ux,uy,uz);

    cams.push_back(p);
    lookats.push_back(l);
    ups.push_back(u);
  }
  updateCamera();

  // now close the file
  pathFile.clear();
  pathFile.close();
}

/**
 * This function loads the camera pose from the given
 * pose file.
 */

void loadPose(int dummy) {
  double euler[3];
  double quat[4];
  double mouseRotX;
  double mouseRotY;
  double mouseRotZ;
  double cangle;
  bool showViewMode;
  bool cameraNavMouseMode;
  double pzoom;
  bool show_path;
  bool show_points;
  bool show_cameras;
  bool show_poses;
  double pointsize;
  int show_fog;
  double fogDensity;
  bool invert;

  // file stream
  std::fstream poseFile;


  // check if other files are open or not
  if(poseFile.is_open()){

    // if open then close the file
    poseFile.clear();
    poseFile.close();

  }

  // open the path file
  poseFile.open(pose_file_name, std::ios::in);
  if(!poseFile.good()) {
    std::cerr << "Error loading file " << pose_file_name << std::endl;
    return;
  }

  // Position
  for (unsigned int i = 0; i < 3; poseFile >> euler[i++]);
  // Orientation
  for (unsigned int i = 0; i < 4; poseFile >> quat[i++]);
  poseFile >> mouseRotX >> mouseRotY >> mouseRotZ >> cangle;
  poseFile >> showViewMode >> cameraNavMouseMode >> pzoom;
  poseFile >> show_points >> show_path >> show_cameras >> pointsize;
  poseFile >> show_fog >> fogDensity >> invert;

  setView(euler, quat, mouseRotX, mouseRotY, mouseRotZ, cangle, showViewMode,
          cameraNavMouseMode, pzoom, show_points, show_path, show_cameras,
          show_poses, pointsize, show_fog, fogDensity, invert);

  poseFile.clear();
  poseFile.close();

}

/**
 * This function saves the current camera pose to a file.
 */

void savePose(int dummy) {
  std::cout << "Save" << std::endl;
  // output file stream
  std::ofstream posefile;

  // open the output file
  posefile.open(pose_file_name, std::ios::out);

  // if file not found then show error
  if(!posefile){
    std::cerr << "Error creating the pose file." << std::endl;
    return;
  }

  // store all the relevant information about the
  // individual camera position in this file.

  posefile << X << " " << Y << " " << Z << std::endl;
  for(int i = 0; i < 4; i++) {
    posefile << quat[i] << " ";
  }
	posefile << mouseRotX << " " <<  mouseRotY << " " << mouseRotZ << " "
		    << cangle << std::endl;
	posefile << showViewMode << " " << cameraNavMouseMode << " "
		    << pzoom << std::endl;
	posefile << show_points << " " << show_path << " "
		    << show_cameras << " " << pointsize << std::endl;
	posefile << show_fog << " " << fogDensity << " " << invert << std::endl;

  // close the file after writing
  posefile.clear();
  posefile.close();

}

std::string suggestImageFileName() {
  return "image" + to_string(imageNr, 3) + ".png";
}

void saveImage(int dummy)
{
  saveImageAt(suggestImageFileName());
}

void saveImageAt(const std::string& imageFileName) {
  glWriteImagePNG(imageFileName.c_str(), factor, 0);
  imageNr++;
}

/**
 * Saves the currently selected points
 *
 */
void saveSelection(int dummy) {
  // output file stream
  std::ofstream selectionfile;

  // open the output file
  selectionfile.open(selection_file_name, std::ios::out);

  // if file not found then show error
  if(!selectionfile){
    std::cerr << "Error creating the selection file." << std::endl;
    return;
  }

  // need higher precision to accurately reload points
  selectionfile.precision(8);
  for(unsigned int i = 0; i < octpts.size(); i++) {
    selectionfile << "# points from scan nr " << i << std::endl;
    // for(int j = 0; j < selected_points[i].size(); j++) {
    for ( std::set<sfloat*>::iterator it = selected_points[i].begin();
        it != selected_points[i].end(); it++) {
      for (unsigned int k = 0; k < pointtype.getPointDim(); k++) {
	   //        selectionfile << selected_points[i][j][k] << " ";
        selectionfile << (*it)[k] << " ";
      }
      selectionfile << std::endl;
    }
  }

  // close the file after writing
  selectionfile.clear();
  selectionfile.close();
}
