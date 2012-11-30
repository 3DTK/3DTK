#
# lastile.py
#
# (c) 2012, Martin Isenburg
# LASSO - rapid tools to catch reality
#
# uses lastile.exe to computes a tiling for the LiDAR points with a
# default tile size of 1000 units (and an optional buffer).
#
# The LiDAR input can be in LAS/LAZ/BIN/TXT/SHP/... format.
# The LiDAR output can be in LAS/LAZ/BIN/TXT/... format.
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
gp.AddMessage("Starting lastile ...")

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

### create the full path to the lastile executable
lastile_path = lastools_path+"\\lastile.exe"

### check if executable exists
if os.path.exists(lastools_path) == False:
    gp.AddMessage("Cannot find lastile.exe at " + lastile_path)
    sys.exit(1)
else:
    gp.AddMessage("Found " + lastile_path + " ...")

### create the command string for lastile.exe
command = [lastile_path]

### maybe use '-verbose' option
if sys.argv[argc-1] == "true":
    command.append("-v")

### add input LiDAR
command.append("-i")
command.append(sys.argv[1])

### maybe use a user-defined tile size
if sys.argv[2] != "1000":
    command.append("-tile_size")
    command.append(sys.argv[2])

### maybe create a buffer around the tiles
if sys.argv[3] != "0":
    command.append("-buffer")
    command.append(sys.argv[3])

### maybe the output will be over 2000 tiles
if sys.argv[4] == "true":
    command.append("-extra_pass")
        
### maybe an output format was selected
if sys.argv[5] != "#":
    if sys.argv[5] == "las":
        command.append("-olas")
    elif sys.argv[5] == "laz":
        command.append("-olaz")
    elif sys.argv[5] == "bin":
        command.append("-obin")
    elif sys.argv[5] == "txt":
        command.append("-otxt")
    elif sys.argv[5] == "xyzi":
        command.append("-otxt")
        command.append("-oparse")
        command.append("xyzi")
    elif sys.argv[5] == "txyzi":
        command.append("-otxt")
        command.append("-oparse")
        command.append("txyzi")

### maybe an output file name was selected
if sys.argv[6] != "#":
    command.append("-o")
    command.append(sys.argv[6])
    
### maybe an output directory was selected
if sys.argv[7] != "#":
    command.append("-odir")
    command.append(sys.argv[7])

### maybe an output appendix was selected
if sys.argv[8] != "#":
    command.append("-odix")
    command.append(sys.argv[8])

### report command string
gp.AddMessage("LAStools command line:")
command_length = len(command)
command_string = str(command[0])
for i in range(1, command_length):
    command_string = command_string + " " + str(command[i])
gp.AddMessage(command_string)

### run command
returncode,output = check_output(command, False)

### report output of lastile
gp.AddMessage(str(output))

### check return code
if returncode != 0:
    gp.AddMessage("Error. lastile failed.")
    sys.exit(1)

### report happy end
gp.AddMessage("Success. lastile done.")
