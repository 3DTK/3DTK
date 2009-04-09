/** 
 * @file 
 * @brief Representation of the parameter of a k-d tree
 * @author Andreas Nuechter. Institute of Computer Science, University of Osnabrueck, Germany.
 */

#ifndef __KDPARAMS_H__
#define __KDPARAMS_H__

/**
 * @brief Contains the intermediate (static) values of a k-d tree or a cached k-d tree
 * 
 * A parameter class for the latter k-d tree.
 * Includes the padding for parallelizetion
 * to avoid cache conflicts.
 **/
class KDParams
{
public:
  /** 
   * pointer to the closest point.  size = 4 bytes of 32 bit machines 
   */
  double *closest;

  /** 
   * distance to the closest point. size = 8 bytes 
   */
  double closest_d2;

  /** 
   * pointer to the point, size = 4 bytes of 32 bit machines 
   */
  double *p;

  /** 
   * expand to 128 bytes to avoid false-sharing, 16 bytes from above + 28*4 bytes = 128 bytes
   */
  int padding[28];
};

#endif
