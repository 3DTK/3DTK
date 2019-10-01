#include "mesh/mesh.h"

using namespace std;

// global
Mesh mesh;

void display();

int main(int argc, char **argv) {
  // load mesh
  if (argc == 2) {
    mesh.load(argv[1]);
  }
  else {
    mesh.load("dat//test//sphere.obj");
  }

  // save mesh
  mesh.save("dat//test//sphere_save.obj");

  // glut related calls
  glutInit(&argc, argv);
  glutInitDisplayMode(GLUT_DEPTH | GLUT_DOUBLE | GLUT_RGBA);
  glutInitWindowPosition(100,100);
  glutInitWindowSize(320,320);
  glutCreateWindow("Mesh Renderer");
  glutDisplayFunc(display);
  glutMainLoop();

  return 0;
}

// draw faces and poly lines
void display() {
  vector<float*> vertices = mesh.getOrderedVertices();
  float *bbox = mesh.getBBox();

  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  float centerX = (bbox[0] + bbox[1]) / 2,
    centerY = (bbox[2] + bbox[3]) / 2,
    centerZ = (bbox[4] + bbox[5]) / 2;

	glLoadIdentity();
  // gluLookAt( centerX, centerY, bbox[5],
	// 		centerX, centerY, centerZ,
	// 		0.0f, -1.0f, 0.0f);

  for (float* face : vertices) {
    glColor3f (0.25, 0.25, 0.25);
    glBegin(GL_TRIANGLES);
      glVertex3f(face[0], face[1], face[2]);
      glVertex3f(face[3], face[4], face[5]);
      glVertex3f(face[6], face[7], face[8]);
    glEnd();
    glColor3f (1.0, 1.0, 1.0);
    glBegin(GL_LINES);
      glVertex3f(face[0], face[1], face[2]);
      glVertex3f(face[3], face[4], face[5]);
      glVertex3f(face[6], face[7], face[8]);
    glEnd();
  }

  glutSwapBuffers();
}
