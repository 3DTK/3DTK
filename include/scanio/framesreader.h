#ifndef __SCAN_IO_FRAMES_READER_H__
#define __SCAN_IO_FRAMES_READER_H__
#include <string>
#include <iostream>
void readFramesAndTransform(std::string dir, int start, int end, int frame, bool use_pose=false, bool reduced=false);
#endif
