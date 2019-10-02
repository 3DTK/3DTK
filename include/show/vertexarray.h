/**
 * @file
 * @brief Representation of an OpenGL vertex array
 * @author Andreas Nuechter. Institute of Computer Science, University of Osnabrueck, Germany.
 */

#ifndef __VERTEXARRAY_H__
#define __VERTEXARRAY_H__

#ifdef _WIN32
#include <windows.h>
#endif

#ifdef WITH_OPENGL
#ifdef __APPLE__
#include <OpenGL/gl.h>
#else
#include <GL/gl.h>
#endif
#else
#include "show/dummygl.h"
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
