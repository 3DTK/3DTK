/**
 * @file PolarPoint.h
 * @brief is a structure containing the theta,phi,refelectance,range,x,y,z
 * @author Andreas Nuechter. Jacobs University Bremen gGmbH, Grmany.
 * @author HamidReza Houshiar. Jacobs University Bremen gGmbH, Germany.
 * @author Darko Makreshanski. Jacobs University Bremen gGmbH, Germany.
 */
#ifndef POLARPOINT_H_
#define POLARPOINT_H_

struct PolarPoint 
{
  float a; //theta
  float b; //phi
  float r; //reflectance
  float d; //range
  float x; 
  float y;
  float z;
};
#endif /* POLARPOINT_H_ */
