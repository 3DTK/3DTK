#!/bin/bash

# Program:
#       This program is written for evaluating the KITTI dataset.
# History:
# 2018/08/01	
# Copyright (C) Shitong Du

# input command line arguments
# $0: shell script name;
# $1: slam6D absolute path; /home/achim/svnwork/slam6d-code
# $2: KITTI dataset absolute path; /media/achim/D46CA2BB6CA2982E/dataset/KITTI
# Please note that evaluation program and ground_truth are also included in $2 path.

#Before using this shell, please copy the groundtruth into $2 path with ground_truth

for sequence in {0..10} 
 do
#enter sequence path including 00-10
 var1=$2
 var11=/
 var2=$(printf "%02d" "$sequence")
 var3=${var1}${var11}${var2}

 cd $var3

#if the frames file exist already,delete them first.
 
echo "the sequence is:" $sequence

 files=$(ls *.3d 2> /dev/null | wc -l)
 if [ "$files" != "0" ] ;then
    echo "scan*.3d files exists already"
 else 
# enter slam6d pacakge
   cd $1
#convert *.bin formate into scan*.3d and calabration
   echo " using the kit2scan program to convert .bin into .3d"
   bin/kitti2scan -s 0 -q $sequence $2  

 fi

  cd $var3

  files=$(ls *.pose 2> /dev/null | wc -l)
  if [ "$files" != "0" ] ;then
    echo "scan*.pose files exists already"
  else 
    echo " creating 0 pose file for scan*.3d"

# creating 0 pose file for scan*.3d

    cfile=`find -name '*.3d'`
    for filename in $cfile
    do
      cfname=`basename -s.3d $filename`
      echo -e "0 0 0\n0 0 0">${cfname}.pose
    done
  fi


 files=$(ls *.frames 2> /dev/null | wc -l)
 if [ "$files" != "0" ] ;then
    echo "delete scan*.frames files exists already"
    rm -r *.frames
 fi

 echo "start to caculate frames files using slam6D"
 
# enter slam6d pacakge
 cd $1

#step1 for icp
 echo "starting step1 for ICP"
 
 bin/slam6D -s 0 -r 10 --octree=1 -u "11;6;-3000.0;3000.0;-150.0;1000;-3000;3000" -i 1500 -d 75 $var3 | tee out

#save frames file for step1
 cd $var3

 if [ ! -d "/st1_frames" ]; then
  mkdir st1_frames
 fi
 var4=/
 var5=st1_frames
 varsp1=${var3}${var4}${var5}
 cd $varsp1

 files=$(ls *.frames 2> /dev/null | wc -l)
 if [ "$files" != "0" ] ;then
    echo "delete scan*.frames files exists already"
    rm -r *.frames
 fi

 cd $var3
 cp ./*.frames ./st1_frames
 echo "The frames files computered by step1 are saved"
 
 
 #convert frames of step1 into ground truth matrix format
 cd $1
 bin/frames2kitti -s 0 -q $sequence $varsp1

#step2 for icp
 echo "start step2 for ICP"
bin/slam6D -s 0 -r 10 --octree=1 -u "11;6;-3000.0;3000.0;-150.0;1000;-3000;3000"  -i 1500 -d 50 --continue -D -1 --DlastSLAM=150 --epsICP=0.000000001 --epsSLAM=0.000000001 -I 500 --cldist=500 --loopsize=100 -L 0 -G 1 $var3 | tee out   
 
#convert frames into ground truth matrix format
 bin/frames2kitti -s 0 -q $sequence $var3
 
  cd $2

  if [ ! -d "/computedpose " ]; then
   mkdir pose_result
  fi
  cp ./$var2/$var2.txt ./computedpose 
done

#evaluation step2 results by comparing with kitti ground truth
 vareva=$2
 vareva1=/
 vareva2=devkit
 varevafi=${vareva}${vareva1}${vareva2}
 cd $varevafi
 g++ -O3 -DNDEBUG -o evaluate_odometry evaluate_odometry.cpp matrix.cpp

 ./evaluate_odometry $2

exit 0

