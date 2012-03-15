#include "veloslam/tracker.h"

Tracker::Tracker(void)
{
	missMatch=false;
	Matched =false;

	matchClusterID=-1;
	colorIdx=-1;
}
 
Tracker::~Tracker(void)
{

}
 