/**
 * @file
 * @brief Representation of an OpenGL vertex array
 * @author Andreas Nuechter. Institute of Computer Science, University of Osnabrueck, Germany.
 */

#ifndef __VERTEXARRAY_H__
#define __VERTEXARRAY_H__

#ifdef _MSC_VER
#include <windows.h>
#endif

#ifdef __APPLE__
#include <OpenGL/gl.h>
#else
#include <GL/gl.h>
#endif


class vertexArray
{
public:
  vertexArray(int _numPointsToRender);
  ~vertexArray();
  
  GLfloat* array;
  GLint    numPointsToRender;
  GLuint   name;
private:
  static GLuint nameCounter;
};

#endif
