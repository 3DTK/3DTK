#include "slam6d/globals.icc"
#include "slam6d/scan.h"
#include "slam6d/basicScan.h"
#include "ros/importer.h"

void scanstruct::setTransform(double *mat, double x, double y, double z) {
  transmat[0] = mat[4];
  transmat[1] = -mat[7];
  transmat[2] = -mat[1];
  transmat[3] = 0.0;

  transmat[4] = -mat[5];
  transmat[5] = mat[8];
  transmat[6] = mat[2];
  transmat[7] = 0.0;

  transmat[8] = -mat[3];
  transmat[9] = mat[6];
  transmat[10] = mat[0];
  transmat[11] = 0.0;

  // translation
  transmat[12] =  -y;
  transmat[13] =  z;
  transmat[14] =  x;
  transmat[15] = 1;

}

//ros::Time starttime;

void timedImporter::on_echo_transformed(echo_type echo)
{
  scanlib::pointcloud::on_echo_transformed(echo);

  static bool firstEcho = true;
  if (firstEcho) {
      target& t(targets[target_count - 1]);

    if (t.time != tmap->firstTimestamp) {
      ROS_WARN("Warning: Timestamps differ between RXP file and bagfile!");
      ROS_WARN("Timestamp of first echo in RXP file: %f", t.time);
      ROS_WARN("First timestamp of time map: %f", tmap->firstTimestamp);
    }

      firstEcho = false;
  }

  if(pointcloud::first == echo || pointcloud::interior == echo) return;
  /*
  if ( pointcloud::first == echo ) {
    p.type = 0;
  } else if ( pointcloud::interior == echo ) {
    p.type = 1;
  } else if ( pointcloud::last == echo ) {
    p.type = 10;
  } else if ( pointcloud::single == echo ){
    p.type = 9;
  }
  */
  haspoints = true;
  target& t(targets[target_count - 1]);

  ros::Time rostime = tmap->toRos(t.time);
  if (remembertime) {
    tf::StampedTransform transform;
    if(!norobot) {
      if(rieglcoord) {
        try{
          cal->getTrajectory()->lookupTransform (mapstring, "/riegl", rostime, transform);
        }catch(tf::ExtrapolationException){ROS_ERROR("couldn't lookup transform map to riegl"); return;}
      } else {
        try{
        cal->getTrajectory()->lookupTransform (mapstring, "/base_link", rostime, transform);
          }catch(tf::ExtrapolationException){ROS_ERROR("couldn't lookup transform map to base_link"); return;}
      }
    } else {
      transform.setIdentity();
    }

    double rotmat[9];
    double X = transform.getOrigin().getX()*100;
    double Y = transform.getOrigin().getY()*100;
    double Z = transform.getOrigin().getZ()*100;
    rotmat[0] = transform.getBasis().getRow(0).getX();
    rotmat[1] = transform.getBasis().getRow(0).getY();
    rotmat[2] = transform.getBasis().getRow(0).getZ();

    rotmat[3] = transform.getBasis().getRow(1).getX();
    rotmat[4] = transform.getBasis().getRow(1).getY();
    rotmat[5] = transform.getBasis().getRow(1).getZ();

    rotmat[6] = transform.getBasis().getRow(2).getX();
    rotmat[7] = transform.getBasis().getRow(2).getY();
    rotmat[8] = transform.getBasis().getRow(2).getZ();
    current->setTransform(rotmat, X, Y, Z);

    remembertime = false;
  }

/*  if(! (pointcloud::first == echo)) {
    return;
  }*/
  if ( sqrt(t.vertex[0]*t.vertex[0] + t.vertex[1]*t.vertex[1] + t.vertex[2]*t.vertex[2]) < 0.8)
    return;
  if (t.deviation > 10.0) {
    return;
  }
  if (start >= (int)counter ) { // ignore points before the beginning
    return;
  }
  if (reducer && !reducer->accept(pcounter++)) {
    return;
  }

  if (use_buffer) {
    double *p = new double[pointtype.getPointDim()];
    for (unsigned int i = 0; i < 3; i++)
      p[i] = t.vertex[i];
    if (pointtype.hasReflectance())
      p[pointtype.getReflectance()] = t.reflectance;
    if (pointtype.hasAmplitude())
      p[pointtype.getAmplitude()] = t.amplitude;
    if (pointtype.hasDeviation())
      p[pointtype.getDeviation()] = t.deviation;
    p[pointtype.getTime()] = t.time;
    pbuffer.push_back(p);
  } else {
    geometry_msgs::PointStamped *p = new geometry_msgs::PointStamped();


    p->header.stamp = rostime;
    p->header.frame_id = "/riegl";
    p->point.x = t.vertex[0];
    p->point.y = t.vertex[1];
    p->point.z = t.vertex[2];
    geometry_msgs::PointStamped *q = new geometry_msgs::PointStamped();
    if (stopandgo) {
      if (rieglcoord) {
        cal->getTrajectory()->transformPoint("/riegl", *p, *q);
      } else {
        cal->getTrajectory()->transformPoint("/base_link", *p, *q);

      }
    } else {
      cal->getTrajectory()->transformPoint("/odom_combined", *p, *q);

    }
    current->points.push_back(q);
    delete p;
  }
}

void timedImporter::on_frame_stop(const frame_stop<iterator_type>& arg) {
  basic_packets::on_frame_stop(arg);
  if (lines ) {
    transform_buffer();
  }

  //TODO newScan();
}

void timedImporter::newScan() {
  if (start >= (int)counter ) { //
    if (haspoints) {
      scanstruct *old = current;
      current = new scanstruct(counter++, !stopandgo );
      delete old;
    }
//    remembertime = true;
    haspoints = false;
    return;
  }

  haspoints = false;

  if (current->hasPoints()) {
    scanstruct *old = current;
    current = new scanstruct(counter++, !stopandgo);

    frameStop(old);
  }

  // get next pose
//  remembertime = true;
}

void timedImporter::transform_buffer() {
  if (pbuffer.empty()) return;
  sensor_msgs::PointCloud buffercloud;
  sensor_msgs::PointCloud *transformedline = new sensor_msgs::PointCloud();

  buffercloud.header.stamp = tmap->toRos( pbuffer[pbuffer.size()/2][ pointtype.getTime()]);
  buffercloud.header.frame_id = "/riegl";
  buffercloud.points.resize(pbuffer.size());
  buffercloud.channels.resize( pointtype.getPointDim() - 4 );

  for (unsigned int i = 0; i < pbuffer.size(); i++) {
    buffercloud.points[i].x = pbuffer[i][0];
    buffercloud.points[i].y = pbuffer[i][1];
    buffercloud.points[i].z = pbuffer[i][2];

    if (pointtype.hasReflectance())  {
      buffercloud.channels[pointtype.getReflectance() - 3].values.push_back(pbuffer[i][pointtype.getReflectance()]);
    }
    if (pointtype.hasAmplitude()) {
      buffercloud.channels[pointtype.getAmplitude() - 3].values.push_back( pbuffer[i][pointtype.getAmplitude()]);
    }
    if (pointtype.hasDeviation()) {
      buffercloud.channels[pointtype.getDeviation() - 3].values.push_back(pbuffer[i][pointtype.getDeviation()]);
    }
    //TODO delete or not delete??
    delete pbuffer[i];
  }
  pbuffer.clear();
  if (stopandgo) {
    if (rieglcoord) {
        try{
      cal->getTrajectory()->transformPointCloud("/riegl", buffercloud, *transformedline);
        }
        catch(tf::ExtrapolationException) {return;}
    } else {
      try{
    cal->getTrajectory()->transformPointCloud("/base_link", buffercloud, *transformedline);

      }
      catch(tf::ExtrapolationException){
        return;
        }

    }
  } else {
      try{
    cal->getTrajectory()->transformPointCloud(mapstring, buffercloud, *transformedline);
     //cout<<transformedline<<endl;
    //cal->getTrajectory()->transformPointCloud("/base_link", buffercloud, *transformedline);
      }
      catch (tf::ExtrapolationException) {ROS_INFO("There is no tranform between %s and /riegl", mapstring.c_str()); return;}
  }
//  listener->transformPointCloud("/riegl", starttime, buffercloud,
/*  listener->transformPointCloud("/base_link", starttime, buffercloud,
                                "/odom_combined", *transformedline);*/
  current->clouds.push_back(transformedline);

  if (linescans)
    newScan();
    remembertime = true;

}



void FileImporter::frameStop(scanstruct *scan) {
  scan->setPath(outputpath);
  ioservice->post(boost::bind(&FileImporter::writeScan, scan));
}

/**
 * function that is executed in a seperate thread to write a scan to file
 *
 */
void FileImporter::writeScan(scanstruct *scan) {
  if (!scan) return;

  std::ofstream o;     // file stream for scan
  string scanFileName = scan->outputpath + "scan"+ to_string(scan->index,3) + ".3d";
  string poseFileName = scan->outputpath + "scan"+ to_string(scan->index,3) + ".pose";
  string framesFileName = scan->outputpath + "scan"+ to_string(scan->index,3) + ".frames";

  int BSIZE = 100;
  int nrpts = 0;
  stringstream buffer;

  // open file
  ROS_INFO("opening scan %d", scan->index);
  o.open(scanFileName.c_str());
  o << std::setprecision(10);

  double tinv[16];
  M4inv(scan->transmat, tinv);

  // write points using the buffer
  ROS_INFO("writing scan %d", scan->index);
  for (unsigned int i = 0; i < scan->points.size(); i++) {
    geometry_msgs::PointStamped *p = scan->points[i];

    double x_neu, y_neu, z_neu, x,y,z;
    x = -100.0 * p->point.y;
    y = 100.0 * p->point.z;
    z = 100.0 * p->point.x;
    if (scan->invert) {
      x_neu = x * tinv[0] + y * tinv[4] + z * tinv[8];
      y_neu = x * tinv[1] + y * tinv[5] + z * tinv[9];
      z_neu = x * tinv[2] + y * tinv[6] + z * tinv[10];
      x = x_neu + tinv[12];
      y = y_neu + tinv[13];
      z = z_neu + tinv[14];
    }

    buffer << x << " " << y << " " << z << endl;
    nrpts++;

    if (nrpts > BSIZE)
    {
      o.write(buffer.str().c_str(), buffer.str().size());
      nrpts = 0;
      buffer.clear();
      buffer.str("");
    }
  }

  for (unsigned int i = 0; i < scan->clouds.size(); i++) {
    sensor_msgs::PointCloud *pc = scan->clouds[i];
    for (unsigned int j = 0; j < pc->points.size(); j++) {
      geometry_msgs::Point32 p = pc->points[j];

      double x_neu, y_neu, z_neu, x,y,z;
      x = -100.0 * p.y;
      y = 100.0 * p.z;
      z = 100.0 * p.x;
      if (scan->invert) {
        x_neu = x * tinv[0] + y * tinv[4] + z * tinv[8];
        y_neu = x * tinv[1] + y * tinv[5] + z * tinv[9];
        z_neu = x * tinv[2] + y * tinv[6] + z * tinv[10];

        x = x_neu + tinv[12];
        y = y_neu + tinv[13];
        z = z_neu + tinv[14];
      }

      buffer << x << " " << y << " " << z;

      for (unsigned int k = 0; k < pc->channels.size(); k++) {
        buffer << " " << pc->channels[k].values[j];
      }
      buffer << endl;

      nrpts++;

      if (nrpts > BSIZE)
      {
        o.write(buffer.str().c_str(), buffer.str().size());
        nrpts = 0;
        buffer.clear();
        buffer.str("");
      }
    }
  }
  o.write(buffer.str().c_str(), buffer.str().size());
  //o.write(buffer, (pos - buffer));
  o.close();

  // pose
  double rP[3];
  double rPT[3];
  Matrix4ToEuler(scan->transmat, rPT, rP);
  o.open(poseFileName.c_str());
  o << rP[0] << " " << rP[1] << " " << rP[2] << endl;
  o << deg(rPT[0]) << " " << deg(rPT[1]) << " " << deg(rPT[2]) << endl;

//  o << "0 0 0\n0 0 0" << std::endl;
  o.flush();
  o.close();

  // frames
  o.open(framesFileName.c_str());


  for (int k = 0; k < 3; k++) {
    for (unsigned int i = 0; i < 16; i++) {
      o << scan->transmat[i] << " ";
    }
    o << "2" << endl;
  }

  o.flush();
  o.close();

  ROS_INFO("Done with scan %u ", scan->index);
  delete scan;
}



void ScanImporter::frameStop(scanstruct *scan) {
  double tinv[16];
  M4inv(scan->transmat, tinv);

  // pose
  double rP[3];
  double rPT[3];
  Matrix4ToEuler(scan->transmat, rPT, rP);
  vector<double *> points;
  // write points using the buffer
//  ROS_INFO("Creating Slam6D Scan %d ", scan->index);
  for (unsigned int i = 0; i < scan->points.size(); i++) {
    geometry_msgs::PointStamped *p = scan->points[i];
    double *point = new double[3];

    double x_neu, y_neu, z_neu;
    point[0] = -100.0 * p->point.y;
    point[1] = 100.0 * p->point.z;
    point[2] = 100.0 * p->point.x;
    if (scan->invert) {
      x_neu = point[0] * tinv[0] + point[1] * tinv[4] + point[2] * tinv[8];
      y_neu = point[0] * tinv[1] + point[1] * tinv[5] + point[2] * tinv[9];
      z_neu = point[0] * tinv[2] + point[1] * tinv[6] + point[2] * tinv[10];
      point[0] = x_neu + tinv[12];
      point[1] = y_neu + tinv[13];
      point[2] = z_neu + tinv[14];
    }

    points.push_back(point);
  }

  points.reserve(scan->clouds.size()*scan->clouds[0]->points.size());
  for (unsigned int i = 0; i < scan->clouds.size(); i++) {
    sensor_msgs::PointCloud *pc = scan->clouds[i];
    for (unsigned int j = 0; j < pc->points.size(); j++) {
      geometry_msgs::Point32 p = pc->points[j];

    double *point = new double[3];
      double x_neu, y_neu, z_neu;
      point[0] = -100.0 * p.y;
      point[1] = 100.0 * p.z;
      point[2] = 100.0 * p.x;
      if (scan->invert) {
        x_neu = point[0] * tinv[0] + point[1] * tinv[4] + point[2] * tinv[8];
        y_neu = point[0] * tinv[1] + point[1] * tinv[5] + point[2] * tinv[9];
        z_neu = point[0] * tinv[2] + point[1] * tinv[6] + point[2] * tinv[10];

        point[0] = x_neu + tinv[12];
        point[1] = y_neu + tinv[13];
        point[2] = z_neu + tinv[14];
      }

      points.push_back(point);

    }
  }

//  srand(0);
//  Scan *sscan = new Scan(rP, rPT, points, 5, -1);
  srandom(0);
  Scan* sscan = new BasicScan(rP, rPT, points );

  scans->push_back(sscan);

//ROS_INFO("Done with scan %u ", scan->index);
  delete scan;
}
void ScanReducer::frameStop(scanstruct *scan) {
  double tinv[16];
  M4inv(scan->transmat, tinv);
  // pose
  double rP[3];
  double rPT[3];
  Matrix4ToEuler(scan->transmat, rPT, rP);

  vector<double *> points;
  // write points using the buffer
//  ROS_INFO("Creating Slam6D Scan %d ", scan->index);
  for (unsigned int i = 0; i < scan->points.size(); i++) {
    geometry_msgs::PointStamped *p = scan->points[i];
    double *point = new double[4];

    double x_neu, y_neu, z_neu;
    point[0] = -100.0 * p->point.y;
    point[1] = 100.0 * p->point.z;
    point[2] = 100.0 * p->point.x;
    if (scan->invert) {
      x_neu = point[0] * tinv[0] + point[1] * tinv[4] + point[2] * tinv[8];
      y_neu = point[0] * tinv[1] + point[1] * tinv[5] + point[2] * tinv[9];
      z_neu = point[0] * tinv[2] + point[1] * tinv[6] + point[2] * tinv[10];
      point[0] = x_neu + tinv[12];
      point[1] = y_neu + tinv[13];
      point[2] = z_neu + tinv[14];
    }
    point[3] = currentID++;

    points.push_back(point);
  }

int cc = 0;
  points.reserve(scan->clouds.size()*scan->clouds[0]->points.size());
  for (unsigned int i = 0; i < scan->clouds.size(); i++) {
    sensor_msgs::PointCloud *pc = scan->clouds[i];
    for (unsigned int j = 0; j < pc->points.size(); j++) {
      geometry_msgs::Point32 p = pc->points[j];

 ////////////////// TODO FIXME !!!!!!!!!!!!!!!!!!
 //cc++;
 //if (cc%10 != 0) continue;
 ////////////////// TODO FIXME !!!!!!!!!!!!!!!!!!

    double *point = new double[4];
      double x_neu, y_neu, z_neu;
      point[0] = -100.0 * p.y;
      point[1] = 100.0 * p.z;
      point[2] = 100.0 * p.x;
      if (scan->invert) {
        x_neu = point[0] * tinv[0] + point[1] * tinv[4] + point[2] * tinv[8];
        y_neu = point[0] * tinv[1] + point[1] * tinv[5] + point[2] * tinv[9];
        z_neu = point[0] * tinv[2] + point[1] * tinv[6] + point[2] * tinv[10];

        point[0] = x_neu + tinv[12];
        point[1] = y_neu + tinv[13];
        point[2] = z_neu + tinv[14];
      }
      point[3] = currentID++;

      points.push_back(point);

    }
  }

//  srand(0);
//  Scan *sscan = new Scan(rP, rPT, points, 5, -1);
  srandom(0);
  PointType pointtype(PointType::USE_TYPE);
  Scan *sscan = new BasicScan(rP, rPT, points);//, voxelsize, nrpts, pointtype );


  cout.precision(15);
  for (int i = 0; i < sscan->size<DataXYZ>("xyz reduced"); i++) {
      DataXYZ pts = sscan->get("xyz reduced");
      reducedIDs.insert(pts[i][3]);
  }



  delete sscan;



//ROS_INFO("Done with scan %u ", scan->index);
  delete scan;

}



void LineScanImporter::frameStop(scanstruct *scan) {
  scan->setPath(outputpath);
  ioservice->post(boost::bind(&LineScanImporter::writeLineScan, scan));
}

/**
 * function that is executed in a seperate thread to write a scan to file
 *
 */
void LineScanImporter::writeLineScan(scanstruct *scan) {
  if (!scan) return;

  std::ofstream o;     // file stream for scan
  string scanFileName = scan->outputpath + "scan"+ to_string(scan->index,3) + ".3d";
  string poseFileName = scan->outputpath + "scan"+ to_string(scan->index,3) + ".pose";
  string framesFileName = scan->outputpath + "scan"+ to_string(scan->index,3) + ".frames";

  stringstream buffer;

  // open file
//  ROS_INFO("opening scan %d", scan->index);
  o.open(scanFileName.c_str());
  o << std::setprecision(10);

  double tinv[16];
  M4inv(scan->transmat, tinv);

  for (unsigned int i = 0; i < scan->clouds.size(); i++) {
    sensor_msgs::PointCloud *pc = scan->clouds[i];
    for (unsigned int j = 0; j < pc->points.size(); j++) {
      geometry_msgs::Point32 p = pc->points[j];

      double x_neu, y_neu, z_neu, x,y,z;
      x = -100.0 * p.y;
      y = 100.0 * p.z;
      z = 100.0 * p.x;
      if (scan->invert) {
        x_neu = x * tinv[0] + y * tinv[4] + z * tinv[8];
        y_neu = x * tinv[1] + y * tinv[5] + z * tinv[9];
        z_neu = x * tinv[2] + y * tinv[6] + z * tinv[10];

        x = x_neu + tinv[12];
        y = y_neu + tinv[13];
        z = z_neu + tinv[14];
      }

      buffer << x << " " << y << " " << z;

      for (unsigned int k = 0; k < pc->channels.size(); k++) {
        buffer << " " << pc->channels[k].values[j];
      }
      buffer << endl;
    }
  }
  o.write(buffer.str().c_str(), buffer.str().size());
  o.close();

  // pose
  double rP[3];
  double rPT[3];
  Matrix4ToEuler(scan->transmat, rPT, rP);
  o.open(poseFileName.c_str());
  o << rP[0] << " " << rP[1] << " " << rP[2] << endl;
  o << deg(rPT[0]) << " " << deg(rPT[1]) << " " << deg(rPT[2]) << endl;
  o << scan->clouds[0]->header.stamp << endl;
  // TODO write covariance as well?
  o.flush();
  o.close();

  // frames
  o.open(framesFileName.c_str());

  for (int k = 0; k < 3; k++) {
    for (unsigned int i = 0; i < 16; i++) {
      o << scan->transmat[i] << " ";
    }
    o << "2" << endl;
  }

  o.flush();
  o.close();

  //ROS_INFO("Done with scan %u ", scan->index);
  delete scan;
}
