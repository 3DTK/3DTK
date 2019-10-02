#pragma once

#include <list>
#include <vector>
#include "veloslam/veloscan.h"
#include "veloslam/tracker.h"

void DumpPointtoFile(cluster &gluData1, std::string filename);
void DumpFeaturetoFile(clusterFeature &glu, std::string filename);
