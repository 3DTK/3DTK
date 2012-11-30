****************************************************************

  The LAStools LiDAR processing Toolbox for ArcGIS 9.3 - 10.1

  (c) 2012 - martin.isenburg@gmail.com

  rapidlasso GmbH - fast tools to catch reality

  legal details: http://rapidlasso.com/LICENSE.txt

****************************************************************

(1) Do not move or copy the "LAStools.tbx" from its original
location ./lastools/ArcGIS_toolbox/LAStools.tbx where it is
after unzipping the lastools.zip distribution. Instead, import
the "LAStools.tbx" at its original location using the "add
Toolbox" mechanism. To get to this location you may have to
make a "folder connection" in ArcGIS to the place where you
unzipped lastools.zip to.

(2) Do not move the ./lastools/ArcGIS_toolbox/scripts or the
./lastools/bin or the folder. The relative path between the
ArcGIS toolbox and the LAStools binaries needs to be preserved.

(3) It it important that there is no space (e.g. " ") in full
name of your LAStools folder path (because of the python scripts).

(4) The space " " limitation is not inherent to python but the
current implementation and can be overcome with strategically
placed quotes around all path+file names. This will get fixed
soon.

This is okay:
C:\lastools
C:\LiDAR\lastools
C:\LiDAR\kickass_tools\lastools
  
This is NOT okay:
C:\Documents and Settings\isenburg\lastools
C:\Documents and Settings\isenburg\My Documents\ArcGIS\lastools
C:\Program Files\lastools
