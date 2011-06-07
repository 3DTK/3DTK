/**
 * @file SuperPixel.h
 * @brief is a structure containing the refelectance,range,x,y,z
 * @author Andreas Nuechter. Jacobs University Bremen gGmbH, Grmany.
 * @author HamidReza Houshiar. Jacobs University Bremen gGmbH, Germany.
 * @author Darko Makreshanski. Jacobs University Bremen gGmbH, Germany.
 */
#ifndef SUPERPIXEL_H_
#define SUPERPIXEL_H_

struct SuperPixel 
{
  float value;//refelectance
  float meta;//range
  float x;
  float y;
  float z;
};
#endif /* SUPERPIXEL_H_ */
