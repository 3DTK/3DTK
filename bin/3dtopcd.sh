#!/bin/sh

for i in *.3d; do
   j=${i%%.3d}.pcd;
   nrlines=`grep -c . $i`
   echo "Welcome $j times with $nrlines"
   echo "# .PCD v.7 - Point Cloud Data file format" > $j
   echo "VERSION .7" >> $j
   echo "FIELDS x y z rgb" >> $j
   echo "SIZE 4 4 4 4" >> $j
   echo "TYPE F F F F" >> $j
   echo "COUNT 1 1 1 1" >> $j
   echo "WIDTH $nrlines " >> $j
   echo "HEIGHT 1" >> $j
   echo "VIEWPOINT 0 0 0 1 0 0 0" >> $j
   echo "POINTS $nrlines " >> $j
   echo "DATA ascii" >> $j
   cat $i >> $j
done
