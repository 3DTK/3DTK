APSCpp == enhanced autopano-sift-c  29 Feb 2008 : 07 Mar 2008 TKSharpless

This is a major upgrade of autopano-sift-c and has version 2.5.0.

New executable autopano-sift-c combines keypoint finding and matching in one 
program. It is built on the autopano-sift-c codebase, and basically delivers 
the same functionality as generatekeys + autopano, plus some new options.  

The most important are the ability to run under hugin as an alternate control
point finder, and the option of converting to stereographic projection before 
finding keypoints.  

The stereographic projection can give better results on wide angle and fisheye 
images.  To construct it, autopano-sift-c needs to know the projection type(s) 
and angular width(s) of the images. There are two ways to give this information.  

When all the images have the same format and hfov, they can be specified with 
new commandline option "--projection".

Or a PanoTools-compatible project file can be given in place of the list of image 
file names at the end of the command, and autopano-sift-c will read the necessary 
data from the "i" lines (or the "o" lines if there are no "i" lines).

The first method allows using the stereographic mode under hugin, by putting
" --projection %f,%v " in hugin's command line template.  The second works with
PanoTools scripts of many kinds, including hugin, PTassembler and PTGui project 
files, but must be run from a command shell.

The output is a .pto file that can be loaded into hugin.  In stereographic mode 
this file has the format and hfov information.

Autopano-sift-c uses a smoother method for reducing image size, which leads to
somewhat more stable control point positions (it also takes a little longer).  

Finally, it uses the much faster ANN kd-tree implementation for keypoint matching
(with the option of using the original code instead, for comparison).

Some notable differences in options from generatekeys/autopano
* the limit on the larger image dimension is --maxdim, not --mindim, and 
  its default value is 1600 instead of 800.
* the default maximum number of control points per image pair is 25 not 16.
* RANSAC filtering is off by default instead of on
* the default kd-tree implementation is ANN.  "--ANNmatch off" can be 
  used to switch back to the original code,

Please see the usage message, obtained by running autopano-sift-c without
arguments, for details on all the options.

There are also some improvements to the existing libsift code.  Besides bug fixes, 
the major change is that most internal image data are now stored as floats rather 
than doubles.  This just about halves the memory demand of the keypoint finder and
allows processing significantly larger images on any given machine.


TODO --

-- find out why enabling RANSAC seems to make the alignments worse, when the 
opposite would be expected, and fix that too.

-- when a hugin project file is input, output a full copy of it with just the 
conttrol points replaced.

