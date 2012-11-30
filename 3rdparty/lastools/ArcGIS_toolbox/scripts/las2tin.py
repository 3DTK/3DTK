#
# las2tin.py
#
# (c) 2012, Martin Isenburg
# LASSO - rapid tools to catch reality
#
# uses las2tin.exe to trianulate LiDAR points into a TIN and store it as
# an ESRI Shapefile of type MultiPatch or as an OBJ file.
#
# The LiDAR input can be in LAS/LAZ/BIN/TXT/SHP/... format.
# The TIN output will be in SHP or OBJ format.
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
gp.AddMessage("Starting las2tin ...")

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

### create the full path to the las2tin executable
las2tin_path = lastools_path+"\\las2tin.exe"

### check if executable exists
if os.path.exists(lastools_path) == False:
    gp.AddMessage("Cannot find las2tin.exe at " + las2tin_path)
    sys.exit(1)
else:
    gp.AddMessage("Found " + las2tin_path + " ...")

### create the command string for las2tin.exe
command = [las2tin_path]

### maybe use '-verbose' option
if sys.argv[argc-1] == "true":
    command.append("-v")

### add input LiDAR
command.append("-i")
command.append(sys.argv[1])

### maybe use user-defined concavity
if sys.argv[2] != "50":
    command.append("-concavity")
    command.append(sys.argv[2])

### what should we triangulate
if sys.argv[3] == "ground points only":
    command.append("-keep_class")
    command.append("2")
    command.append("-extra_pass")
elif sys.argv[3] == "ground and keypoints":
    command.append("-keep_class")
    command.append("2")
    command.append("8")
    command.append("-extra_pass")
elif sys.argv[3] == "ground and buildings":
    command.append("-keep_class")
    command.append("2")
    command.append("6")
    command.append("-extra_pass")
elif sys.argv[3] == "ground and vegetation":
    command.append("-keep_class")
    command.append("2")
    command.append("3")
    command.append("4")
    command.append("5")
    command.append("-extra_pass")
elif sys.argv[3] == "ground and objects":
    command.append("-keep_class")
    command.append("2")
    command.append("3")
    command.append("4")
    command.append("5")
    command.append("6")
    command.append("-extra_pass")
            
### do we have lakes
if sys.argv[4] != "#":
    command.append("-lakes")
    command.append(sys.argv[4])

### do we have creeks
if sys.argv[5] != "#":
    command.append("-creeks")
    command.append(sys.argv[5])

### this is where the output arguments start
out = 6

### maybe an output format was selected
if sys.argv[out] != "#":
    command.append("-o" + sys.argv[out])

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

### report output of las2tin
gp.AddMessage(str(output))

### check return code
if returncode != 0:
    gp.AddMessage("Error. las2tin failed.")
    sys.exit(1)

### report happy end
gp.AddMessage("Success. las2tin done.")
