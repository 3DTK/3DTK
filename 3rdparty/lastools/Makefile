all:
	cd laslib && make
	cd src && make
#	cd src_full && make

clean:
	cd laslib && make clean
	cd src && make clean
#	cd src_full && make clean

clobber:
	cd laslib && make clobber
	cd src && make clobber
#	cd src_full && make clobber
	cd bin && rm -rf lasinfo laszip lasprecision las2txt txt2las las2las lasdiff lasmerge lasthin las2shp shp2las las2iso las2tin lasboundary lasview las2dem
