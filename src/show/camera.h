/**
 * @file camera.h
 * @brief Header file for the class camera
 *
 */

#ifndef __CAMERA_H__
#define __CAMERA_H__

typedef struct {
  float r;
  float g;
  float b;
} Color;

/**
 * Class Camera
 * \brief This is the class camera.
 */

 
class Camera {

private:
  float x;
  float y;
  float z;
  float r_x, r_y, r_z;
  float angle;
  float cam_size;
  float focal_length;
  float focal_x;
  float focal_y;
  float focal_z;
  float view_width;
  float fx, fy, fz;
  Color camcolor;

public:
  Camera();
  ~Camera();
  void drawCamera(bool);
  void addCamera(float,float,float,float,float,float,float,float,float,float,
			  float newflength = 200.0f,
			  float newsize = 20.0f,
			  float newvwidth = 100.0f);
  void deleteCamera();
  void moveCamera( float,float,float,float, float newsize = 20.0f);
  
  float getX();
  float getY();
  float getZ();
  float getAngle();
  float getCamSize();
  float getRX();
  float getRY();
  float getRZ();
  void setCamSize(float);
  void setX(float);
  void setY(float);
  void setZ(float);
  void setRX(float);
  void setRY(float);
  void setRZ(float);
  void setAngle(float);
  Color getColor();
  void setColor(Color);
  void setRotate(float,float,float,float);
  float getFocalLength();
  void setFocalLength(float);
  float getFX();
  float getFY();
  float getFZ();
  void setFX(float);
  void setFY(float);
  void setFZ(float);
  float getF_x();
  float getF_y();
  float getF_z();
  void setF_x(float);
  void setF_y(float);
  void setF_z(float);
};

#include "camera.icc"

#endif
