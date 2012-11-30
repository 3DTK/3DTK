#
# lasheight.py
#
# (c) 2012, Martin Isenburg
# LASSO - rapid tools to catch reality
#
# uses lasheight to compute the height of LiDAR points above the points
# classified as ground or above another given set of points or DEM.
#
# The LiDAR input can be in LAS/LAZ/BIN/TXT/SHP/... format.
# The LiDAR output can be in LAS/LAZ/BIN/TXT format.
#
# for licensing details see http://rapidlasso.com/download/LICENSE.txt
#

import sys, os, arcgisscripting, subprocess

def return_classification(classification):
    if (classification == "created, never classified (0)"):
        return "0"
    if (classification == "unclassified (1)"):
        return "1"
    if (classification == "ground (2)"):
        return "2"
    if (classification == "low vegetation (3)"):
        return "3"
    if (classification == "medium vegetation (4)"):
        return "4"
    if (classification == "high vegetation (5)"):
        return "5"
    if (classification == "building (6)"):
        return "6"
    if (classification == "low point (7)"):
        return "7"
    if (classification == "keypoint (8)"):
        return "8"
    if (classification == "water (9)"):
        return "9"
    if (classification == "high point (10)"):
        return "10"
    if (classification == "(11)"):
        return "11"
    if (classification == "overlap point (12)"):
        return "12"
    if (classification == "(13)"):
        return "13"
    if (classification == "(14)"):
        return "14"
    if (classification == "(15)"):
        return "15"
    if (classification == "(16)"):
        return "16"
    if (classification == "(17)"):
        return "17"
    if (classification == "(18)"):
        return "18"
    return "unknown"

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
gp.AddMessage("Starting lasheight ...")

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

### create the full path to the lasheight executable
lasheight_path = lastools_path+"\\lasheight.exe"

### check if executable exists
if os.path.exists(lastools_path) == False:
    gp.AddMessage("Cannot find lasheight.exe at " + lasheight_path)
    sys.exit(1)
else:
    gp.AddMessage("Found " + lasheight_path + " ...")

### create the command string for lasheight.exe
command = [lasheight_path]

### maybe use '-verbose' option
if sys.argv[argc-1] == "true":
    command.append("-v")

### add input LiDAR
command.append("-i")
command.append(sys.argv[1])

### maybe use ground points from external file
if sys.argv[2] != "#":
    command.append("-ground_points")
    command.append(sys.argv[2])
        
### else maybe use points with a different classification as ground
elif sys.argv[3] != "#":
    command.append("-class")
    command.append(return_classification(sys.argv[3]))

### maybe we should do replace the z coordinate with the height value
if sys.argv[4] == "true":
    command.append("-replace_z")

### maybe we should drop all points above    
if sys.argv[5] != "#":
    command.append("-drop_above")
    command.append(sys.argv[5])

### maybe we should drop all points below    
if sys.argv[6] != "#":
    command.append("-drop_below")
    command.append(sys.argv[6])

### this is where the output arguments start
out = 7

### maybe an output format was selected
if sys.argv[out] != "#":
    if sys.argv[out] == "las":
        command.append("-olas")
    elif sys.argv[out] == "laz":
        command.append("-olaz")
    elif sys.argv[out] == "bin":
        command.append("-obin")
    elif sys.argv[out] == "xyz":
        command.append("-otxt")
    elif sys.argv[out] == "xyzi":
        command.append("-otxt")
        command.append("-oparse")
        command.append("xyzi")
    elif sys.argv[out] == "txyzi":
        command.append("-otxt")
        command.append("-oparse")
        command.append("txyzi")

### maybe an output file name was selected
if sys.argv[out+1] != "#":
    command.append("-o")
    command.append(sys.argv[out+1])

### maybe an output directory was selected
if sys.argv[out+2] != "#":
    command.append("-odir")
    command.append(sys.argv[out+2])

### maybe an output appendix was selected
if sys.argv[out+3] != "#":
    command.append("-odix")
    command.append(sys.argv[out+3])

### report command string
gp.AddMessage("LAStools command line:")
command_length = len(command)
command_string = str(command[0])
for i in range(1, command_length):
    command_string = command_string + " " + str(command[i])
gp.AddMessage(command_string)

### run command
returncode,output = check_output(command, False)

### report output of lasheight
gp.AddMessage(str(output))

### check return code
if returncode != 0:
    gp.AddMessage("Error. lasheight failed.")
    sys.exit(1)

### report happy end
gp.AddMessage("Success. lasheight done.")
