# couple of specific examples
# first convert to binary polar point cloud object

bin/readscan -o /home/nuechter/dat/bremen_city/scan000.txt.ppc /home/nuechter/dat/bremen_city/scan000.txt 
bin/readscan -o /home/nuechter/dat/bremen_city/scan001.txt.ppc /home/nuechter/dat/bremen_city/scan001.txt 
bin/readscan -o /home/nuechter/dat/bremen_city/scan002.txt.ppc /home/nuechter/dat/bremen_city/scan002.txt 
bin/readscan -o /home/nuechter/dat/bremen_city/scan003.txt.ppc /home/nuechter/dat/bremen_city/scan003.txt 
bin/readscan -o /home/nuechter/dat/bremen_city/scan004.txt.ppc /home/nuechter/dat/bremen_city/scan004.txt 
bin/readscan -o /home/nuechter/dat/bremen_city/scan005.txt.ppc /home/nuechter/dat/bremen_city/scan005.txt 
bin/readscan -o /home/nuechter/dat/bremen_city/scan006.txt.ppc /home/nuechter/dat/bremen_city/scan006.txt 
bin/readscan -o /home/nuechter/dat/bremen_city/scan007.txt.ppc /home/nuechter/dat/bremen_city/scan007.txt 
bin/readscan -o /home/nuechter/dat/bremen_city/scan008.txt.ppc /home/nuechter/dat/bremen_city/scan008.txt 
bin/readscan -o /home/nuechter/dat/bremen_city/scan009.txt.ppc /home/nuechter/dat/bremen_city/scan009.txt 
bin/readscan -o /home/nuechter/dat/bremen_city/scan010.txt.ppc /home/nuechter/dat/bremen_city/scan010.txt 
bin/readscan -o /home/nuechter/dat/bremen_city/scan011.txt.ppc /home/nuechter/dat/bremen_city/scan011.txt 
bin/readscan -o /home/nuechter/dat/bremen_city/scan012.txt.ppc /home/nuechter/dat/bremen_city/scan012.txt 


# generate panorama maps and images (only maps are used)
# (this generates files like scan001.txt.ppc_2880x800.map)

bin/panoramacreator -m -j -s 2880x800 /home/nuechter/dat/bremen_city/scan000.txt.ppc 
bin/panoramacreator -m -j -s 2880x800 /home/nuechter/dat/bremen_city/scan001.txt.ppc 
bin/panoramacreator -m -j -s 2880x800 /home/nuechter/dat/bremen_city/scan002.txt.ppc 
bin/panoramacreator -m -j -s 2880x800 /home/nuechter/dat/bremen_city/scan003.txt.ppc 
bin/panoramacreator -m -j -s 2880x800 /home/nuechter/dat/bremen_city/scan004.txt.ppc 
bin/panoramacreator -m -j -s 2880x800 /home/nuechter/dat/bremen_city/scan005.txt.ppc 
bin/panoramacreator -m -j -s 2880x800 /home/nuechter/dat/bremen_city/scan006.txt.ppc 
bin/panoramacreator -m -j -s 2880x800 /home/nuechter/dat/bremen_city/scan007.txt.ppc 
bin/panoramacreator -m -j -s 2880x800 /home/nuechter/dat/bremen_city/scan008.txt.ppc 
bin/panoramacreator -m -j -s 2880x800 /home/nuechter/dat/bremen_city/scan009.txt.ppc 
bin/panoramacreator -m -j -s 2880x800 /home/nuechter/dat/bremen_city/scan010.txt.ppc 
bin/panoramacreator -m -j -s 2880x800 /home/nuechter/dat/bremen_city/scan011.txt.ppc 
bin/panoramacreator -m -j -s 2880x800 /home/nuechter/dat/bremen_city/scan012.txt.ppc 


# generate features

bin/generatesiftfeatures -x /home/nuechter/dat/bremen_city/scan000.txt.ppc_2880x800.map.xml -i /home/nuechter/dat/bremen_city/scan000.txt.ppc_2880x800.map 
bin/generatesiftfeatures -x /home/nuechter/dat/bremen_city/scan001.txt.ppc_2880x800.map.xml -i /home/nuechter/dat/bremen_city/scan001.txt.ppc_2880x800.map 
bin/generatesiftfeatures -x /home/nuechter/dat/bremen_city/scan002.txt.ppc_2880x800.map.xml -i /home/nuechter/dat/bremen_city/scan002.txt.ppc_2880x800.map 
bin/generatesiftfeatures -x /home/nuechter/dat/bremen_city/scan003.txt.ppc_2880x800.map.xml -i /home/nuechter/dat/bremen_city/scan003.txt.ppc_2880x800.map 
bin/generatesiftfeatures -x /home/nuechter/dat/bremen_city/scan004.txt.ppc_2880x800.map.xml -i /home/nuechter/dat/bremen_city/scan004.txt.ppc_2880x800.map  
bin/generatesiftfeatures -x /home/nuechter/dat/bremen_city/scan005.txt.ppc_2880x800.map.xml -i /home/nuechter/dat/bremen_city/scan005.txt.ppc_2880x800.map 
bin/generatesiftfeatures -x /home/nuechter/dat/bremen_city/scan006.txt.ppc_2880x800.map.xml -i /home/nuechter/dat/bremen_city/scan006.txt.ppc_2880x800.map 
bin/generatesiftfeatures -x /home/nuechter/dat/bremen_city/scan007.txt.ppc_2880x800.map.xml -i /home/nuechter/dat/bremen_city/scan007.txt.ppc_2880x800.map 
bin/generatesiftfeatures -x /home/nuechter/dat/bremen_city/scan008.txt.ppc_2880x800.map.xml -i /home/nuechter/dat/bremen_city/scan008.txt.ppc_2880x800.map 
bin/generatesiftfeatures -x /home/nuechter/dat/bremen_city/scan009.txt.ppc_2880x800.map.xml -i /home/nuechter/dat/bremen_city/scan009.txt.ppc_2880x800.map 
bin/generatesiftfeatures -x /home/nuechter/dat/bremen_city/scan010.txt.ppc_2880x800.map.xml -i /home/nuechter/dat/bremen_city/scan010.txt.ppc_2880x800.map 
bin/generatesiftfeatures -x /home/nuechter/dat/bremen_city/scan011.txt.ppc_2880x800.map.xml -i /home/nuechter/dat/bremen_city/scan011.txt.ppc_2880x800.map 
bin/generatesiftfeatures -x /home/nuechter/dat/bremen_city/scan012.txt.ppc_2880x800.map.xml -i /home/nuechter/dat/bremen_city/scan012.txt.ppc_2880x800.map 


# you can match all at once but I don't recommend this
# bin/matchsiftfeatures -o /home/nuechter/dat/bremen_city/all.matches /home/nuechter/dat/bremen_city/*.xml

# match pairwise only 
bin/matchsiftfeatures -o /home/nuechter/dat/bremen_city/1.2.matches /home/nuechter/dat/bremen_city/scan001.txt.ppc_2880x800.map.xml /home/nuechter/dat/bremen_city/scan002.txt.ppc_2880x800.map.xml 
bin/matchsiftfeatures -o /home/nuechter/dat/bremen_city/2.3.matches /home/nuechter/dat/bremen_city/scan002.txt.ppc_2880x800.map.xml /home/nuechter/dat/bremen_city/scan003.txt.ppc_2880x800.map.xml 
bin/matchsiftfeatures -o /home/nuechter/dat/bremen_city/3.4.matches /home/nuechter/dat/bremen_city/scan003.txt.ppc_2880x800.map.xml /home/nuechter/dat/bremen_city/scan004.txt.ppc_2880x800.map.xml 
bin/matchsiftfeatures -o /home/nuechter/dat/bremen_city/4.5.matches /home/nuechter/dat/bremen_city/scan004.txt.ppc_2880x800.map.xml /home/nuechter/dat/bremen_city/scan005.txt.ppc_2880x800.map.xml 
bin/matchsiftfeatures -o /home/nuechter/dat/bremen_city/5.6.matches /home/nuechter/dat/bremen_city/scan005.txt.ppc_2880x800.map.xml /home/nuechter/dat/bremen_city/scan006.txt.ppc_2880x800.map.xml 
bin/matchsiftfeatures -o /home/nuechter/dat/bremen_city/6.7.matches /home/nuechter/dat/bremen_city/scan006.txt.ppc_2880x800.map.xml /home/nuechter/dat/bremen_city/scan007.txt.ppc_2880x800.map.xml 
bin/matchsiftfeatures -o /home/nuechter/dat/bremen_city/7.8.matches /home/nuechter/dat/bremen_city/scan007.txt.ppc_2880x800.map.xml /home/nuechter/dat/bremen_city/scan008.txt.ppc_2880x800.map.xml 
bin/matchsiftfeatures -o /home/nuechter/dat/bremen_city/8.9.matches /home/nuechter/dat/bremen_city/scan008.txt.ppc_2880x800.map.xml /home/nuechter/dat/bremen_city/scan009.txt.ppc_2880x800.map.xml 
bin/matchsiftfeatures -o /home/nuechter/dat/bremen_city/9.10.matches /home/nuechter/dat/bremen_city/scan009.txt.ppc_2880x800.map.xml /home/nuechter/dat/bremen_city/scan010.txt.ppc_2880x800.map.xml 
bin/matchsiftfeatures -o /home/nuechter/dat/bremen_city/10.11.matches /home/nuechter/dat/bremen_city/scan010.txt.ppc_2880x800.map.xml /home/nuechter/dat/bremen_city/scan011.txt.ppc_2880x800.map.xml 
bin/matchsiftfeatures -o /home/nuechter/dat/bremen_city/11.12.matches /home/nuechter/dat/bremen_city/scan011.txt.ppc_2880x800.map.xml /home/nuechter/dat/bremen_city/scan012.txt.ppc_2880x800.map.xml 
bin/matchsiftfeatures -o /home/nuechter/dat/bremen_city/12.0.matches /home/nuechter/dat/bremen_city/scan012.txt.ppc_2880x800.map.xml /home/nuechter/dat/bremen_city/scan000.txt.ppc_2880x800.map.xml 


# register scans. input matches maps, and then -d for minimum amount
# of inliers, -t for inlier error threshold and -x for amount of
# iterations per pairwise registration
bin/registerscans -m /home/nuechter/dat/bremen_city/1.2.matches -m /home/nuechter/dat/bremen_city/2.3.matches -m /home/nuechter/dat/bremen_city/3.4.matches -m /home/nuechter/dat/bremen_city/4.5.matches -m /home/nuechter/dat/bremen_city/5.6.matches -m /home/nuechter/dat/bremen_city/6.7.matches -m /home/nuechter/dat/bremen_city/7.8.matches -m /home/nuechter/dat/bremen_city/8.9.matches -m /home/nuechter/dat/bremen_city/9.10.matches -m /home/nuechter/dat/bremen_city/10.11.matches -m /home/nuechter/dat/bremen_city/11.12.matches -m /home/nuechter/dat/bremen_city/12.0.matches /home/nuechter/dat/bremen_city/*800.map -d 10 -t 0.5 -x 200000


# end of registration


# you can visualize result with:
# (-m 20 to include only every 20th point, -c random color for each scan
#  -r would be rainbow map for the height of the points)
bin/visualizeregistrations -m 20 -c -p /home/nuechter/dat/bremen_city/scan001.txt.ppc -p /home/nuechter/dat/bremen_city/scan002.txt.ppc -p /home/nuechter/dat/bremen_city/scan003.txt.ppc -p /home/nuechter/dat/bremen_city/scan004.txt.ppc -p /home/nuechter/dat/bremen_city/scan005.txt.ppc -p /home/nuechter/dat/bremen_city/scan006.txt.ppc -p /home/nuechter/dat/bremen_city/scan007.txt.ppc -p /home/nuechter/dat/bremen_city/scan008.txt.ppc -p /home/nuechter/dat/bremen_city/scan009.txt.ppc -p /home/nuechter/dat/bremen_city/scan010.txt.ppc -p /home/nuechter/dat/bremen_city/scan011.txt.ppc -p /home/nuechter/dat/bremen_city/scan012.txt.ppc -p /home/nuechter/dat/bremen_city/scan000.txt.ppc

# if you want to visualize registrations more often and don't want to
# wait so much to read each object, you can reduce the polar point
# cloud objects with e.g.
# "reduceppc -m 20 -o scan001.txt-20.ppc scan001.txt.ppc"
