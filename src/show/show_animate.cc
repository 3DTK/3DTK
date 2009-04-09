void calcLookAtPath()
{
  PointXY temp ;

  // if camera list is empty then return
  if(cam_list.size() == 0){
    	return;
  }

  // note: remember that the focal point we are trying to interpolate
  // here has the following coordinates in the camera
  // (x,y,z+focal_length) where
  //  x = cameras x position
  //  y = cameras y position
  //  z = cameras z position
  //  focal_length = camera's focal length
  
  // to interpolate with the xy coordinate.
  for (unsigned int i = 0; i < cam_list.size(); i++) {
    temp.x = cam_list.at(i)->getFX();
    temp.y = cam_list.at(i)->getFY();
    if(i < lookat_listXY.size()){
	 lookat_listXY.at(i).x = temp.x;
	 lookat_listXY.at(i).y = temp.y;
    } else {
	 lookat_listXY.push_back(temp);
    }
  }

  
  // to interpolate with the xz coordinate.
  for (unsigned int i = 0; i < cam_list.size(); i++) {
    temp.x = cam_list.at(i)->getFX();
    temp.y = cam_list.at(i)->getFZ();
    
    if(i < lookat_listXZ.size()) {
	 lookat_listXZ.at(i).x = temp.x;
	 lookat_listXZ.at(i).y = temp.y;
    } else {
	 lookat_listXZ.push_back(temp);
    }
  }

  // now get the nurbs path for the individual xy and xz plane
  lookat_vectorX = cam_nurbs_path.getNurbsPath(lookat_listXY, 500);
  lookat_vectorZ = cam_nurbs_path.getNurbsPath(lookat_listXZ, 500);
}

void calcPath()
{
  int points1 = 0, points2 = 0;
  int point;
  PointXY temp ;
 
  // if camera list is empty then return
  if (cam_list.size() == 0) {
    return;
  }

  // to interpolate with the xy coordinate.
  for(unsigned int i = 0;i < cam_list.size(); i++){
    temp.x = cam_list.at(i)->getX();
    temp.y = cam_list.at(i)->getY();
    if(i<path_listXY.size()){
	 path_listXY.at(i).x = temp.x;
	 path_listXY.at(i).y = temp.y;
    }else{
	 path_listXY.push_back(temp);
    }
  }

  // to interpolate with the xz coordinate.
  for(unsigned int i=0;i<cam_list.size();i++){
    temp.x = cam_list.at(i)->getX();
    temp.y = cam_list.at(i)->getZ();
    if(i<path_listXZ.size()){
	 path_listXZ.at(i).x = temp.x;
	 path_listXZ.at(i).y = temp.y;
    }else{
	 path_listXZ.push_back(temp);
    }
  }

  points1 = calcNoOfPoints(path_listXY);
  points2 = calcNoOfPoints(path_listXZ);
 
  (points1 > points2) ? point = points1 : point = points2;
 
  // now get the nurbs path for the individual xy and xz plane
  path_vectorX = cam_nurbs_path.getNurbsPath(path_listXY,point);
  path_vectorZ = cam_nurbs_path.getNurbsPath(path_listXZ,point);
}

/**
 * This function does the path animation.
 */
void pathAnimate1(int i)
{

  //if there are cameras present then do the animation
  if(!cam_list.empty()){

    //some matrices to store values
    double *up = 0;
    double matrix[16];
    double matrix_quat[4];
    double direction[3];

    //if both path and scan matching needs to be animated then
    if(animate_both || (!enable_camera_angle)){

	 //the lookat direction matrix would be
	 direction[0] = path_vectorX.at(i).x - path_vectorX.at(i+3).x;
	 direction[1] = path_vectorX.at(i).y - path_vectorX.at(i+3).y;
	 direction[2] = path_vectorZ.at(i).y - path_vectorZ.at(i+3).y;
	 
    }else{
	 
	 
	 //another nurbs path for the direction of the camera to lookat is created
	 //using the calckLookAtPath
	 calcLookAtPath();

	 //now we have the nurbs path for the lookat points thus

	 //our lookat direction matrix would be
	 direction[0] = path_vectorX.at(i).x - lookat_vectorX.at(i).x;
	 direction[1] = path_vectorX.at(i).y - lookat_vectorX.at(i).y;
	 direction[2] = path_vectorZ.at(i).y - lookat_vectorZ.at(i).y;
	 
	 
    }
    
   
    
    //now call lookAt function to calculate our lookat matrix
    lookAt(direction, 0,0,0,matrix, up);
    // lookAt(direction, lookat_vectorX.at(i).x , lookat_vectorX.at(i).y, lookat_vectorZ.at(i).y,matrix,up);
    //convert the matrix to quaternion
    Matrix4ToQuaternion(matrix, matrix_quat);

    //normalize it
    QuatNormalize(matrix_quat);

    //copy it to our global quaternion matrix
    memcpy(quat, matrix_quat, sizeof(quat));

    // haven't figured out why but it seems that the normal values are the
    // reflection in the y=x plane hence all values have been negated here
    // have to figure out why. (reason found)

    //(reason) Here instead of the camera moving forward we are
    //moving the entire world backward to get an impression
    //of us moving forward. hence all values have been negated.

    X = -path_vectorX.at(i).x;
    Z = -path_vectorZ.at(i).y;
    Y = -path_vectorX.at(i).y;
    
    //signal about repainting the animation scene
    haveToUpdate = 6;

  }
}


//------------------------------------------------------------------------------------------

/**
 * This function saves the path drawn on the
 * screen to a file.
 */

void savePath(int dummy) {

  //output file stream
  ofstream pathfile;

  //open the output file
  pathfile.open(path_file_name, ios::out);
   
  //if file not found then show error
  if(!pathfile){
    cerr << "Error creating the path file." << endl;
    return;
  }

  //store all the relevant information about the
  //individual cameras in this file. however, start
  //with the total number of cameras in the first line
  
  pathfile << cam_list.size() << endl;
  for(unsigned int i =0; i< cam_list.size();i++) {
    pathfile << cam_list.at(i)->getX() << endl;
    pathfile << cam_list.at(i)->getY() << endl;
    pathfile << cam_list.at(i)->getZ() << endl;
    pathfile << cam_list.at(i)->getRX() << endl;
    pathfile << cam_list.at(i)->getRY() << endl;
    pathfile << cam_list.at(i)->getRZ() << endl;
    pathfile << cam_list.at(i)->getAngle() << endl;
    pathfile << cam_list.at(i)->getFocalLength() << endl;
    pathfile << cam_list.at(i)->getFX() << endl;
    pathfile << cam_list.at(i)->getFY() << endl;
    pathfile << cam_list.at(i)->getFZ() << endl;
  }

  //close the file after writing
  pathfile.clear();
  pathfile.close();
  

}

 
//--------------------------------------------------------------------------------------------

/**
 * This function loads the path from the given
 * path file.
 */

void loadPath(int dummy) {

  //some temporary variables
  unsigned int length;
  float val1, val2, val3;
  float tmp_rx, tmp_ry, tmp_rz, tmp_angle, tmp_flength;
  float tmp_fx, tmp_fy, tmp_fz;

  Camera *newcam;

  //buffer variable
  char buffer[2048];

  //file stream
  fstream pathFile;


  //check if other files are open or not
  if(pathFile.is_open()){

    //if open then close the file
    pathFile.clear();
    pathFile.close();
       
  }

  //clear the camlist first
  cam_list.clear();

  //open the path file
  pathFile.open(path_file_name, ios::in);

 
  //read the first line containing info
  //about no of cameras in the file
  pathFile.getline(buffer,2048);
  length = atoi(buffer);

 
  //now for each camera extract the stored
  //information
  for(unsigned int i=0;i<length;i++) {
    pathFile.getline(buffer,2048);
    val1 = atof(buffer);
        
    pathFile.getline(buffer,2048);
    val2 = atof(buffer);
     
    pathFile.getline(buffer,2048);
    val3 = atof(buffer);

    pathFile.getline(buffer,2048);
    tmp_rx = atof(buffer);

    pathFile.getline(buffer,2048);
    tmp_ry = atof(buffer);

    pathFile.getline(buffer,2048);
    tmp_rz = atof(buffer);

    pathFile.getline(buffer,2048);
    tmp_angle = atof(buffer);

    pathFile.getline(buffer,2048);
    tmp_flength = atof(buffer);

    pathFile.getline(buffer,2048);
    tmp_fx = atof(buffer);

    pathFile.getline(buffer,2048);
    tmp_fy = atof(buffer);

    pathFile.getline(buffer,2048);
    tmp_fz = atof(buffer);
 
    newcam  = new Camera;

    //feed the information to create a new
    //camera with those values
    newcam->addCamera( tmp_fx, tmp_fy, tmp_fz, tmp_angle, tmp_rx, tmp_ry, tmp_rz, val1, val2, val3, tmp_flength);

    //now store it in the cam list vector
    cam_list.push_back(newcam);
  }

  //reset the cam_choice spinner
  cam_spinner->set_int_limits( 1, cam_list.size() );
  
  //now close the file
  pathFile.clear();
  pathFile.close();

}

