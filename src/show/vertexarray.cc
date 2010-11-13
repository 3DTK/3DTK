#include "show/vertexarray.h"

GLuint vertexArray::nameCounter = 0;

vertexArray::vertexArray(int _numPointsToRender) :
  numPointsToRender(_numPointsToRender)
{
    array = new GLfloat[numPointsToRender * 3];
    nameCounter++;
    name = nameCounter;
}

vertexArray::~vertexArray()
{
  delete [] array;
}
