-------------------------------------------------------------------

To compile the project simply call "make". This will configure slam6d
using the default settings. If you wish to configure the project using
custom settings do: "make config". This command requires ccmake be
installed on your system.  Alternatively you may change into the build
directory ".build" and configure the project with your preferred cmake
configurator, i.e.:

cd .build && cmake -i ../

For Microsoft Windows, use the cmake-gui application provided by cmake
to configure and generate project files for the appropriate version of
Microsoft Visual Studio C++ of your system. Use the INSTALL target to
built the entire project.  Executables (and .dll's) will then reside
in the "windows" folder. For running the binaries you need to install
the proper redistributable package.

Some Boost libraries (graph, regex, serialization, filesystem,
interprocess) are needed to compile the slam6D program.

If you are using Debian just do:

  aptitude install libboost-graph-dev libboost-regex-dev libboost-serialization-dev freeglut3-dev libxmu-dev libxi-dev

or, if you are still using Debian stable (lenny):

  aptitude install libboost-graph1.35-dev libboost-regex1.35-dev libboost-serialization1.35-dev freeglut3-dev libxmu-dev libxi-dev

for Ubuntu this would be:

  sudo apt-get install libboost-all-dev libcv-dev freeglut3-dev libxmu-dev libxi-dev

SuSE users please use yast2 for installing the missing packages

Additionally you need some external tools (exemplarily for Ubuntu):

  sudo apt-get install imagemagick ffmpeg libx264-120

-------------------------------------------------------------------

For a detailed explanation of the programm, its usage, etc., please
refer to the high level documentation doc/documentation_HL.pdf
(esp. sections 4-6, for starters).

IMPORTANT:
Take care to register scans first (bin/slam6D) before trying to
display them (bin/show), and think about using the point reduction
(see section 6) for a much faster scan matching result. Extremely
large scans might need to be reduced (using bin/scan_red) before
registration. This will write reduced scans in the uos format into a
directory "reduced" in the data directory.

Three example scans are included in the dat directory, several
larger data sets can be downloaded from the data repository,
http://kos.informatik.uni-osnabrueck.de/3Dscans/
(Alternatively, click on the "Data Repository" link on this project's
web site on Sourceforge, http://slam6d.sourceforge.net/)

EXAMPLES:
(using the data set in the slam6d repository)
bin/slam6D -m 500 -R 5 -d 25.0 --metascan dat
bin/show dat

(using the data set in the slam6d repository)
bin/slam6D --max=500 -r 10.2 -i 20 --metascan dat
bin/show dat

(using hannover1.tgz from http://kos.informatik.uni-osnabrueck.de/3Dscans/)
bin/slam6D -s 1 -e 65 -r 10 -i 100 -d 75 -D 250 --epsICP=0.00001 
           -I 50 --cldist=750 -L 0 -G 1 /home/nuechter/dat/dat_hannover1
bin/show -s 1 -e 65 /home/nuechter/dat/dat_hannover1

(using hannover2.tgz from http://kos.informatik.uni-osnabrueck.de/3Dscans/)
bin/slam6D -q -r 10 -f rts -s 23 -d 75 -L 4 --cldist=1500 -G 1 -D -1
           --DlastSLAM 250 --graphDist 200 -I 50 hannover2
bin/show -f rts -s 23 hannover2

(using kvarntorp_mine.tgz (dat_mine1) form http://kos.informatik.uni-osnabrueck.de/3Dscans/)
bin/slam6D -s 1 -e 76 -r 10 -m 3000 -d 50 -i 1000 --epsICP=0.000001 
           -I 50 -D 75 --clpairs=5000 -f old /home/nuechter/dat/dat_mine1/
bin/show -s 1 -e 76 -m 3000 -f old /home/nuechter/dat/dat_mine1/

(using bremen_city.zip from http://kos.informatik.uni-osnabrueck.de/3Dscans/)
bin/scan_red -s 0 -e 12 -r OCTREE -v 10 --octree 0 -f RIEGL_TXT /home/nuechter/dat/bremen_city
bin/slam6D -a 2 -q /home/nuechter/dat/bremen_city/reduced -f uos -d 150  
            -s 0 -e 12 --anim=1 -n /home/nuechter/dat/bremen_city/bremen.net
		  -G 1 -D 100 -i 0 -I 50 -p --epsSLAM=0.0 
bin/show -s 0 -e 12 /home/nuechter/dat/bremen_city/reduced

(using UniKoblenz_CampusTour3_OsnabrueckFormat.tar.gz from
http://kos.informatik.uni-osnabrueck.de/3Dscans/)
bin/slam6D -s 1 -e 320 -r 20 -i 300 --epsICP=0.000001 -d 45 -D 45
           -f uos --algo=2 -l 10 -L 4 -I 100 -G 1
            /home/nuechter/dat/UniKoblenz_CampusTour3_OsnabrueckFormat/
bin/show -s 1 -e 320 -f uos /home/nuechter/dat/UniKoblenz_CampusTour3_OsnabrueckFormat/		  
-------------------------------------------------------------------

For detecting planes compile with the WITH_PLANE option.
Adapt the settings in bin/hough.cfg for your data set.

EXAMPLE: (using the data set in the slam6d repository, no modification
of bin/hough.cfg necessary)

bin/planes -s 0 dat
bin/show -s 0 -e 0 dat -l dat/planes/planes.list

(using bremen_city.zip from http://kos.informatik.uni-osnabrueck.de/3Dscans/)
adapt these settings in bin/hough.cfg:
RhoNum              500          
RhoMax              5000  
MaxPointPlaneDist   50.0 
MinPlaneSize        50
PointDist           100.0  
/bin/planes -f riegl_txt -s 0 /home/nuechter/dat/bremen_city/ -r 50 -O 1 -m 5000
/bin/show -s 0 -e 0 /home/nuechter/dat/bremen_city/ -f riegl_txt -l dat/planes/planes.list -r 10 -O 1 -m 5000

-------------------------------------------------------------------
The IO relevant parameters -f(ormat), -s(tart), -e(nd) can be omitted
in slam6D and show if a 'format' file exists in the directory, which
contains key=value lines (spaces are trimmed automatically) for
format, start, end with the same values as in the commandline. These
format-file parameters will be overwritten by commandline parameters
so that the format-file will provide the right IO type and full index
range and the user can overwrite the index range as he sees fit.

-------------------------------------------------------------------
A reference manual can be found in doc/refman.pdf resp.
doc/html/index.html (type in 'make docu' to compile the doxygen
documentation for the HTML files).
