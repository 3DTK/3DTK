/*
 * PointCloud_gl.h
 *
 *  Created on: Mar 14, 2010
 *      Author: darko
 */

#ifndef POINTCLOUD_GL_H_
#define POINTCLOUD_GL_H_

#include "PointCloud.h"
#include "opengl_framework/Object_gl.h"
#include "Coord.h"
#include <GL/glut.h>

class PointCloud_gl: public Object_gl {
private:
    PointCloud *pc;
    Coord color;
    Coord translate;
public:

	bool rainbowcolor;

	PointCloud_gl(PointCloud *pc) {
		rainbowcolor = false;
		this->pc = pc;
		color = Coord(1,1,1);
		translate = Coord(0,0,0);
	}
	virtual ~PointCloud_gl() {}

	void show()
	{
		
		PointC *ps = pc->getData();

		long c = pc->getLength();
        glBegin(GL_POINTS);
        double r;
        for(long l = 0;l < c;l++){
            r = ps[l].r;
            if (!rainbowcolor) {
	            glColor3f(r*color.x, r*color.y, r*color.z);
	        } else {
	        	Coord c;
	        	float v = ((ps[l].y + translate.x)/1.5 + 20);
	        	if (v < 0) v = 0;
	        	if (v > 100) v = 100;
				if (v < 16.66) {
					c.z = v / 16.66;
				} else if (v < 33.33) {
					c.z = 1;
					c.y = (v - 16.66) / 16.66;
				} else if (v < 50) {
					c.z = (50 - v) / 16.66;
					c.y = 1;
				} else if (v < 66.66) {
					c.x = (v - 50) / 16.66;
					c.y = 1;
				} else if (v < 83.33) {
					c.y = (83.33 - v) / 16.66;
					c.x = 1;
				} else {
					c.x = (100 - v) / 16.66;
				}
				c = c * r;
				glColor3f(c.x,c.y,c.z);
	        }
            glVertex3f(ps[l].x + translate.x, ps[l].y + translate.y, ps[l].z + translate.z);
        }
        glEnd();
    }

    ;
    Coord getColor() const
    {
        return color;
    }

    Coord getTranslate() const
    {
        return translate;
    }

    void setColor(Coord color)
    {
        this->color = color;
    }

    void setTranslate(Coord transform)
    {
        this->translate = transform;
    }

};

#endif /* POINTCLOUD_GL_H_ */
