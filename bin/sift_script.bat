@ECHO OFF
REM
REM first convert to binary polar point cloud object
REM
bin\readscan.exe -o d:\bremen_city\scan000.txt.ppc d:\bremen_city\scan000.txt 
bin\readscan.exe -o d:\bremen_city\scan001.txt.ppc d:\bremen_city\scan001.txt 
bin\readscan.exe -o d:\bremen_city\scan002.txt.ppc d:\bremen_city\scan002.txt 
bin\readscan.exe -o d:\bremen_city\scan003.txt.ppc d:\bremen_city\scan003.txt 
bin\readscan.exe -o d:\bremen_city\scan004.txt.ppc d:\bremen_city\scan004.txt 
bin\readscan.exe -o d:\bremen_city\scan005.txt.ppc d:\bremen_city\scan005.txt 
bin\readscan.exe -o d:\bremen_city\scan006.txt.ppc d:\bremen_city\scan006.txt 
bin\readscan.exe -o d:\bremen_city\scan007.txt.ppc d:\bremen_city\scan007.txt 
bin\readscan.exe -o d:\bremen_city\scan008.txt.ppc d:\bremen_city\scan008.txt 
bin\readscan.exe -o d:\bremen_city\scan009.txt.ppc d:\bremen_city\scan009.txt 
bin\readscan.exe -o d:\bremen_city\scan010.txt.ppc d:\bremen_city\scan010.txt 
bin\readscan.exe -o d:\bremen_city\scan011.txt.ppc d:\bremen_city\scan011.txt 
bin\readscan.exe -o d:\bremen_city\scan012.txt.ppc d:\bremen_city\scan012.txt 
REM
REM generate panorama maps and images (only maps are used)
REM (this generates files like scan001.txt.ppc_2880x800.map)
REM
bin\panoramacreator.exe -m -s 2880x800 d:\bremen_city\scan000.txt.ppc 
bin\panoramacreator.exe -m -s 2880x800 d:\bremen_city\scan001.txt.ppc 
bin\panoramacreator.exe -m -s 2880x800 d:\bremen_city\scan002.txt.ppc 
bin\panoramacreator.exe -m -s 2880x800 d:\bremen_city\scan003.txt.ppc 
bin\panoramacreator.exe -m -s 2880x800 d:\bremen_city\scan004.txt.ppc 
bin\panoramacreator.exe -m -s 2880x800 d:\bremen_city\scan005.txt.ppc 
bin\panoramacreator.exe -m -s 2880x800 d:\bremen_city\scan006.txt.ppc 
bin\panoramacreator.exe -m -s 2880x800 d:\bremen_city\scan007.txt.ppc 
bin\panoramacreator.exe -m -s 2880x800 d:\bremen_city\scan008.txt.ppc 
bin\panoramacreator.exe -m -s 2880x800 d:\bremen_city\scan009.txt.ppc 
bin\panoramacreator.exe -m -s 2880x800 d:\bremen_city\scan010.txt.ppc 
bin\panoramacreator.exe -m -s 2880x800 d:\bremen_city\scan011.txt.ppc 
bin\panoramacreator.exe -m -s 2880x800 d:\bremen_city\scan012.txt.ppc 
REM
REM generate features
REM
bin\generatesiftfeatures.exe -x d:\bremen_city\scan000.txt.ppc_2880x800.map.xml -i d:\bremen_city\scan000.txt.ppc_2880x800.map 
bin\generatesiftfeatures.exe -x d:\bremen_city\scan001.txt.ppc_2880x800.map.xml -i d:\bremen_city\scan001.txt.ppc_2880x800.map 
bin\generatesiftfeatures.exe -x d:\bremen_city\scan002.txt.ppc_2880x800.map.xml -i d:\bremen_city\scan002.txt.ppc_2880x800.map 
bin\generatesiftfeatures.exe -x d:\bremen_city\scan003.txt.ppc_2880x800.map.xml -i d:\bremen_city\scan003.txt.ppc_2880x800.map 
bin\generatesiftfeatures.exe -x d:\bremen_city\scan004.txt.ppc_2880x800.map.xml -i d:\bremen_city\scan004.txt.ppc_2880x800.map  
bin\generatesiftfeatures.exe -x d:\bremen_city\scan005.txt.ppc_2880x800.map.xml -i d:\bremen_city\scan005.txt.ppc_2880x800.map 
bin\generatesiftfeatures.exe -x d:\bremen_city\scan006.txt.ppc_2880x800.map.xml -i d:\bremen_city\scan006.txt.ppc_2880x800.map 
bin\generatesiftfeatures.exe -x d:\bremen_city\scan007.txt.ppc_2880x800.map.xml -i d:\bremen_city\scan007.txt.ppc_2880x800.map 
bin\generatesiftfeatures.exe -x d:\bremen_city\scan008.txt.ppc_2880x800.map.xml -i d:\bremen_city\scan008.txt.ppc_2880x800.map 
bin\generatesiftfeatures.exe -x d:\bremen_city\scan009.txt.ppc_2880x800.map.xml -i d:\bremen_city\scan009.txt.ppc_2880x800.map 
bin\generatesiftfeatures.exe -x d:\bremen_city\scan010.txt.ppc_2880x800.map.xml -i d:\bremen_city\scan010.txt.ppc_2880x800.map 
bin\generatesiftfeatures.exe -x d:\bremen_city\scan011.txt.ppc_2880x800.map.xml -i d:\bremen_city\scan011.txt.ppc_2880x800.map 
bin\generatesiftfeatures.exe -x d:\bremen_city\scan012.txt.ppc_2880x800.map.xml -i d:\bremen_city\scan012.txt.ppc_2880x800.map 
REM
REM you can match all at once but I don't recommend this
REM bin\matchsiftfeatures -o d:\bremen_city\all.matches d:\bremen_city\*.xml
REM
REM match pairwise only
REM
bin\matchsiftfeatures.exe -o d:\bremen_city\1.2.matches d:\bremen_city\scan001.txt.ppc_2880x800.map.xml d:\bremen_city\scan002.txt.ppc_2880x800.map.xml 
bin\matchsiftfeatures.exe -o d:\bremen_city\2.3.matches d:\bremen_city\scan002.txt.ppc_2880x800.map.xml d:\bremen_city\scan003.txt.ppc_2880x800.map.xml 
bin\matchsiftfeatures.exe -o d:\bremen_city\3.4.matches d:\bremen_city\scan003.txt.ppc_2880x800.map.xml d:\bremen_city\scan004.txt.ppc_2880x800.map.xml 
bin\matchsiftfeatures.exe -o d:\bremen_city\4.5.matches d:\bremen_city\scan004.txt.ppc_2880x800.map.xml d:\bremen_city\scan005.txt.ppc_2880x800.map.xml 
bin\matchsiftfeatures.exe -o d:\bremen_city\5.6.matches d:\bremen_city\scan005.txt.ppc_2880x800.map.xml d:\bremen_city\scan006.txt.ppc_2880x800.map.xml 
bin\matchsiftfeatures.exe -o d:\bremen_city\6.7.matches d:\bremen_city\scan006.txt.ppc_2880x800.map.xml d:\bremen_city\scan007.txt.ppc_2880x800.map.xml 
bin\matchsiftfeatures.exe -o d:\bremen_city\7.8.matches d:\bremen_city\scan007.txt.ppc_2880x800.map.xml d:\bremen_city\scan008.txt.ppc_2880x800.map.xml 
bin\matchsiftfeatures.exe -o d:\bremen_city\8.9.matches d:\bremen_city\scan008.txt.ppc_2880x800.map.xml d:\bremen_city\scan009.txt.ppc_2880x800.map.xml 
bin\matchsiftfeatures.exe -o d:\bremen_city\9.10.matches d:\bremen_city\scan009.txt.ppc_2880x800.map.xml d:\bremen_city\scan010.txt.ppc_2880x800.map.xml 
bin\matchsiftfeatures.exe -o d:\bremen_city\10.11.matches d:\bremen_city\scan010.txt.ppc_2880x800.map.xml d:\bremen_city\scan011.txt.ppc_2880x800.map.xml 
bin\matchsiftfeatures.exe -o d:\bremen_city\11.12.matches d:\bremen_city\scan011.txt.ppc_2880x800.map.xml d:\bremen_city\scan012.txt.ppc_2880x800.map.xml 
bin\matchsiftfeatures.exe -o d:\bremen_city\12.0.matches d:\bremen_city\scan012.txt.ppc_2880x800.map.xml d:\bremen_city\scan000.txt.ppc_2880x800.map.xml 
REM
REM register scans. input matches maps, and then -d for minimum amount
REM of inliers, -t for inlier error threshold and -x for amount of
REM iterations per pairwise registration
REM
bin\registerscans.exe -m d:\bremen_city\1.2.matches -m d:\bremen_city\2.3.matches -md:\bremen_city\3.4.matches -m d:\bremen_city\4.5.matches -m d:\bremen_city\5.6.matches -m d:\bremen_city\6.7.matches -m d:\bremen_city\7.8.matches -m d:\bremen_city\8.9.matches -m d:\bremen_city\9.10.matches -m d:\bremen_city\10.11.matches -m d:\bremen_city\11.12.matches -m d:\bremen_city\12.0.matches d:\bremen_city\*800.map -d 10 -t 0.5 -x 200000
REM
REM end of registration
REM
REM you can visualize result with (linux only):
REM  (-m 20 to include only every 20th point, -c random color for each scan
REM   -r would be rainbow map for the height of the points)
REM bin\visualizeregistrations -m 20 -c -p d:\bremen_city\scan001.txt.ppc -p d:\bremen_city\scan002.txt.ppc -p d:\bremen_city\scan003.txt.ppc -p d:\bremen_city\scan004.txt.ppc -p d:\bremen_city\scan005.txt.ppc -p d:\bremen_city\scan006.txt.ppc -p d:\bremen_city\scan007.txt.ppc -p d:\bremen_city\scan008.txt.ppc -p d:\bremen_city\scan009.txt.ppc -p d:\bremen_city\scan010.txt.ppc -p d:\bremen_city\scan011.txt.ppc -p d:\bremen_city\scan012.txt.ppc -p d:\bremen_city\scan000.txt.ppc
