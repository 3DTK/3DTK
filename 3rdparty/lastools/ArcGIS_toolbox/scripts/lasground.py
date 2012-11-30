#
# lasground.py
#
# (c) 2012, Martin Isenburg
# LASSO - rapid tools to catch reality
#
# uses lasground.exe to extracts the bare-earth by classifying LIDAR
# points into ground (class = 2) and non-ground points (class = 1).
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
gp.AddMessage("Starting lasground ...")

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

### create the full path to the lasground executable
lasground_path = lastools_path+"\\lasground.exe"

### check if executable exists
if os.path.exists(lastools_path) == False:
    gp.AddMessage("Cannot find lasground.exe at " + lasground_path)
    sys.exit(1)
else:
    gp.AddMessage("Found " + lasground_path + " ...")

### create the command string for lasground.exe
command = [lasground_path]

### maybe use '-verbose' option
if sys.argv[argc-1] == "true":
    command.append("-v")

### add input LiDAR
command.append("-i")
command.append(sys.argv[1])

### maybe horizontal feet
if sys.argv[2] == "true":
    command.append("-feet")

### maybe vertical feet
if sys.argv[3] == "true":
    command.append("-elevation_feet")

### what type of terrain do we have
if sys.argv[4] == "city or warehouses":
    command.append("-city")
elif sys.argv[4] == "towns or flats":
    command.append("-town")
elif sys.argv[4] == "metropolis":
    command.append("-metro")

### what granularity should we operate with
if sys.argv[5] == "fine":
    command.append("-fine")
elif sys.argv[5] == "extra fine":
    command.append("-extra_fine")
elif sys.argv[5] == "ultra fine":
    command.append("-ultra_fine")

### maybe we should ignore/preserve some existing classifications when classifying
if sys.argv[6] != "#":
    command.append("-ignore_class")
    command.append(return_classification(sys.argv[6]))

### maybe we should ignore/preserve some more existing classifications when classifying
if sys.argv[7] != "#":
    command.append("-ignore_class")
    command.append(return_classification(sys.argv[7]))

### this is where the output arguments start
out = 8

### maybe an output format was selected
if sys.argv[out] != "#":
    if sys.argv[out] == "las":
        command.append("-olas")
    elif sys.argv[out] == "laz":
        command.append("-olaz")
    elif sys.argv[out] == "bin":
        command.append("-obin")
    elif sys.argv[out] == "xyzc":
        command.append("-otxt")
        command.append("-oparse")
        command.append("xyzc")
    elif sys.argv[out] == "xyzci":
        command.append("-otxt")
        command.append("-oparse")
        command.append("xyzci")
    elif sys.argv[out] == "txyzc":
        command.append("-otxt")
        command.append("-oparse")
        command.append("txyzc")
    elif sys.argv[out] == "txyzci":
        command.append("-otxt")
        command.append("-oparse")
        command.append("txyzci")

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

### report output of lasground
gp.AddMessage(str(output))

### check return code
if returncode != 0:
    gp.AddMessage("Error. lasground failed.")
    sys.exit(1)

### report happy end
gp.AddMessage("Success. lasground done.")
