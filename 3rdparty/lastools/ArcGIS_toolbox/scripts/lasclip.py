#
# lasclip.py
#
# (c) 2012, Martin Isenburg
# LASSO - rapid tools to catch reality
#
# uses lasclip.exe to clip (or classify) LiDAR points against polygons
# such as building  footprints, tree crown descriptions, or flight swath
# boundaries.
#
# The LiDAR input can be in LAS/LAZ/BIN/TXT/SHP/... format.
# The clip polygon can be in SHP/TXT format.
# The LiDAR output can be in LAS/LAZ/BIN/TXT format.
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
gp.AddMessage("Starting lasclip ...")

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

### create the full path to the lasclip executable
lasclip_path = lastools_path+"\\lasclip.exe"

### check if executable exists
if os.path.exists(lastools_path) == False:
    gp.AddMessage("Cannot find lasclip.exe at " + lasclip_path)
    sys.exit(1)
else:
    gp.AddMessage("Found " + lasclip_path + " ...")

### create the command string for lasclip.exe
command = [lasclip_path]

### maybe use '-verbose' option
if sys.argv[argc-1] == "true":
    command.append("-v")

### add input LiDAR
command.append("-i")
command.append(sys.argv[1])

### add input polygon
command.append("-poly")
command.append(sys.argv[2])

### maybe invert clipping operation
if sys.argv[3] == "true":
    command.append("-interior")

### maybe classify instead of clip
if sys.argv[4] == "true":
    command.append("-classify")
    command.append(sys.argv[5])
        
### maybe an output format was selected
if sys.argv[6] != "#":
    if sys.argv[6] == "las":
        command.append("-olas")
    elif sys.argv[6] == "laz":
        command.append("-olaz")
    elif sys.argv[6] == "bin":
        command.append("-obin")
    elif sys.argv[6] == "txt":
        command.append("-otxt")
    elif sys.argv[6] == "xyzi":
        command.append("-otxt")
        command.append("-oparse")
        command.append("xyzi")
    elif sys.argv[6] == "txyzi":
        command.append("-otxt")
        command.append("-oparse")
        command.append("txyzi")

### maybe an output file name was selected
if sys.argv[7] != "#":
    command.append("-o")
    command.append(sys.argv[7])

### maybe an output directory was selected
if sys.argv[8] != "#":
    command.append("-odir")
    command.append(sys.argv[8])

### maybe an output appendix was selected
if sys.argv[9] != "#":
    command.append("-odix")
    command.append(sys.argv[9])

### report command string
gp.AddMessage("LAStools command line:")
command_length = len(command)
command_string = str(command[0])
for i in range(1, command_length):
    command_string = command_string + " " + str(command[i])
gp.AddMessage(command_string)

### run command
returncode,output = check_output(command, False)

### report output of lasclip
gp.AddMessage(str(output))

### check return code
if returncode != 0:
    gp.AddMessage("Error. lasclip failed.")
    sys.exit(1)

### report happy end
gp.AddMessage("Success. lasclip done.")
