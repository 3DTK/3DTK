# 3DTK - Planes
**This project is part of the [3D Toolkit (3DTK)](http://threedtk.de)**

## Build Instructions
To compile the project simply call `make`. This will configure slam6d using the default settings. If you wish to configure the project using custom settings do: `make config`. This command requires _ccmake_ be installed on your system.  Alternatively you may change into the build directory `.build` and configure the project with your preferred cmake configurator, i.e.:
```
cd .build && cmake -i ../
```
For **Microsoft Windows**, use the _cmake-gui_ application provided by _cmake_ to configure and generate project files for the appropriate version of _Microsoft Visual Studio C++_ of your system. Use the `INSTALL`  target to built the entire project.  Executables (and .dll's) will then reside in the `windows` folder.


## Getting Started

For a detailed explanation of the functionality of the programm, please refer to:
Dorit Borrmann, Jan Elseberg, Kai Lingemann, and Andreas Nüchter. _The 3D Hough Transform for Plane Detection in Point Clouds - A Review and A new Accumulator Design_, Journal 3D Research, Springer, Volume 2, Number 2, March 2011.

One example scans is included in the dat directory, several larger data sets can be downloaded from the [data repository](http://kos.informatik.uni-osnabrueck.de/3Dscans/).

##### Important
For detecting planes compile `slam6D` with the `WITH_SHAPE_DETECTION` option. Adapt the settings in `bin/hough.cfg` for your data set.

Extremely large scans or scans with large differences in point distribution might need to be reduced before registration. Use the `-r` option to use an octree based reduction of the scan. It is highly recommended to use the -O option as well for reduction to chose points randomly (instead of the center) from the octree voxels.

For a list and description of all the parameters run without any parameters `bin/planes`.

### Examples:
##### Using the data set in the slam6d repository:
```
bin/planes dat
```
###### Using octree based reduction of the scan with voxel size 10 and 3 points from each voxel:
```
bin/planes -s 0 -r 10 -O 3 dat
```
###### Using octree based reduction of the scan with voxel size 10 using the center of each voxel:
```
bin/planes -s 0 -r 10 dat
```
The generated output is written to the directory specified in the config file in `bin/hough.cfg` (default: `dat/planes`). The output contains a file planes.list that lists the files containing the detected planes. Each file with the name `planeXXX.3d` (with `XXX` numbers in increasing order) specifies the 3D points of the convex hull of the largest cluster detected on the plane.

To view the result with the viewer from `slam6d`:
```
bin/show -s 0 -e 0 dat -l dat/planes/planes.list
```
