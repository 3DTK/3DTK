#
# lasview.py
#
# (c) 2012, Martin Isenburg
# LASSO - rapid tools to catch reality
#
# uses lasview.exe to 
#
# The LiDAR input can be in LAS/LAZ/BIN/TXT/SHP/ASC... format.
#
# for licensing details see http://rapidlasso.com/download/LICENSE.txt
#

import sys, os, arcgisscripting, subprocess

def check_output(command,console):
    if console == True:
        process = subprocess.Popen(command)
    else:
        process = subprocess.Popen(command, shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, universal_newlines=True)
    output,error = process.communicate()
    returncode = process.poll()
    return returncode,output 

### create the geoprocessor object
gp = arcgisscripting.create(9.3)

### report that something is happening
gp.AddMessage("Starting lasview ...")

### get number of arguments
argc = len(sys.argv)

### report arguments (for debug)
#gp.AddMessage("Arguments:")
#for i in range(0, argc):
#    gp.AddMessage("[" + str(i) + "]" + sys.argv[i])

### get the path to the LAStools binaries
lastools_path = os.path.dirname(os.path.dirname(os.path.dirname(sys.argv[0])))+"\\bin"

### check if path exists
if os.path.exists(lastools_path) == False:
    gp.AddMessage("Cannot find .\lastools\bin at " + lastools_path)
    sys.exit(1)
else:
    gp.AddMessage("Found " + lastools_path + " ...")

### create the full path to the lasview executable
lasview_path = lastools_path+"\\lasview.exe"

### check if executable exists
if os.path.exists(lastools_path) == False:
    gp.AddMessage("Cannot find lasview.exe at " + lasview_path)
    sys.exit(1)
else:
    gp.AddMessage("Found " + lasview_path + " ...")

### create the command string for lasview.exe
command = [lasview_path]

### maybe use '-verbose' option
if sys.argv[argc-1] == "true":
    command.append("-v")

### add input LiDAR
command.append("-i")
command.append(sys.argv[1])

### maybe display different number of points
if sys.argv[2] != "2000000":
    command.append("-points")
    command.append(sys.argv[2])

### which points to consider
if sys.argv[3] == "first returns":
    command.append("-only_first")
elif sys.argv[3] == "last returns":
    command.append("-only_last")
elif sys.argv[3] == "multi returns":
    command.append("-only_multi")
elif sys.argv[3] == "single returns":
    command.append("-only_single")
elif sys.argv[3] == "ground":
    command.append("-ground")
elif sys.argv[3] == "buildings":
    command.append("-buildings")
elif sys.argv[3] == "vegetation":
    command.append("-vegetation")
elif sys.argv[3] == "objects":
    command.append("-objects")
elif sys.argv[3] == "ground and buildings":
    command.append("-ground_buildings")
elif sys.argv[3] == "ground and vegetation":
    command.append("-ground_vegetation")
elif sys.argv[3] == "ground and objects":
    command.append("-ground_objects")

### how to color
if sys.argv[4] == "height ramp 1":
    command.append("-color_by_height1")
elif sys.argv[4] == "height ramp 2":
    command.append("-color_by_height2")
elif sys.argv[4] == "classification":
    command.append("-color_by_classification")
elif sys.argv[4] == "rgb":
    command.append("-color_by_rgb")
elif sys.argv[4] == "flight line":
    command.append("-color_by_flight_line")
elif sys.argv[4] == "intensity":
    command.append("-color_by_intensity")
elif sys.argv[4] == "number returns":
    command.append("-color_by_returns")

### maybe show control points
if sys.argv[5] != "#":
    command.append("-cp")
    command.append(sys.argv[5])

### maybe add the control point parse string
if sys.argv[6] != "xyz":
    command.append("-cp_parse")
    command.append(sys.argv[6])

### report command string
gp.AddMessage("LAStools command line:")
command_length = len(command)
command_string = str(command[0])
for i in range(1, command_length):
    command_string = command_string + " " + str(command[i])
gp.AddMessage(command_string)

### run command
returncode,output = check_output(command, False)

### report output of lasview
gp.AddMessage(str(output))

### check return code
if returncode != 0:
    gp.AddMessage("Error. lasview failed.")
    sys.exit(1)

### report happy end
gp.AddMessage("Success. lasview done.")
