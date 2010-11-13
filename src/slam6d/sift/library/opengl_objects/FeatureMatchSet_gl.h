#ifndef __FEATURE_MATCH_SET_GL_H__
#define __FEATURE_MATCH_SET_GL_H__

#include "FeatureMatchSet.h"
#include "PanoramaMap.h"
#include "opengl_framework/Object_gl.h"
#include <assert.h>
#include <iostream>
#include <math.h>
#include <GL/gl.h>
#include <GL/glut.h>

using namespace std;

//TODO: this should be connected with PanoramaMap_gl because it depends on it

enum FeatureMatchSet_gl_mode {
	FMS_first,
	FMS_second
};

class FeatureMatchSet_gl : public Object_gl
{
	public:
	
		static const float scale = 0.01;
	
		FeatureMatchSet_gl (FeatureMatchSet *set, PanoramaMap *map, FeatureMatchSet_gl_mode mode ) {
			assert(set);
			assert(map);
			string scid;
			switch(mode) {
				case FMS_first:
					scid = set->firstscan;
					break;
				case FMS_second:
					scid = set->secondscan;
					break;
				default:
					break;
			}
			if (map->scanid != scid) {
				cerr << "Scan ids's don't match for FeatureMatchSet_gl constructor\n";
				throw 1;
			}
			this->map = map;
			this->set = set;
			this->mode = mode;
		}

		virtual ~FeatureMatchSet_gl ();

		void show() {
			assert(set);
			
			int w = map->width;
			int h = map->height;
			int msize = set->matches.size();
			for (int i = 0 ; i < msize ; i++) {
				float ratio = (float) (i + 1) / (float) msize;
				float rat1 = (float) ((int) (sqrt(ratio) * 100000000.0) % 5000) / 5000.0;
				float rat2 = (float) ((int) (sqrt(ratio) * 10000000.0) % 5000) / 5000.0;
				float rat3 = (float) ((int) (sqrt(ratio) * 1000000.0) % 5000) / 5000.0;
				rat1/=3;
				rat2/=3;
				rat3/=3;
				rat1+=0.66;
				rat2+=0.66;
				rat3+=0.66;
				glColor3f(rat1, rat2, rat3);
				FeatureBase feat;
				if (mode == FMS_first) {
					feat = set->matches[i].first;
				} else {
					feat = set->matches[i].second;
				}
				double xscale = feat.x*scale;
				double yscale = feat.y*scale;
//				cout << "BLA" << xscale << " " << yscale << endl;
				glPushMatrix();
				glTranslatef(feat.x * scale, (h - feat.y) * scale, w / 15000.0);
				glutSolidSphere(w / 30000.0, 10, 10);
				glPopMatrix();
				glVertex3f(xscale, yscale,0);
				glVertex3f(xscale, yscale+scale,0);
				glVertex3f(xscale+scale, yscale+scale,0);
				glVertex3f(xscale+scale, yscale,0);
			}
			
		}

		FeatureMatchSet_gl_mode mode;
		FeatureMatchSet *set;
		PanoramaMap *map;
	private:
		/* data */
};

#endif /* __FEATURE_MATCH_SET_H__ */

