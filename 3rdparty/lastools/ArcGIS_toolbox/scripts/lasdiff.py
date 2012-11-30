#
# lasdiff.py
#
# (c) 2012, Martin Isenburg
# LASSO - rapid tools to catch reality
#
# uses lasdiff.exe to 
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
gp.AddMessage("Starting lasdiff ...")

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

### create the full path to the lasdiff executable
lasdiff_path = lastools_path+"\\lasdiff.exe"

### check if executable exists
if os.path.exists(lastools_path) == False:
    gp.AddMessage("Cannot find lasdiff.exe at " + lasdiff_path)
    sys.exit(1)
else:
    gp.AddMessage("Found " + lasdiff_path + " ...")

### create the command string for lasdiff.exe
command = [lasdiff_path]

### maybe use '-verbose' option
if sys.argv[argc-1] == "true":
    command.append("-v")

### add first input LiDAR
command.append("-i")
command.append(sys.argv[1])

### maybe add second input LiDAR
if sys.argv[2] != "#":
    command.append("-i")
    command.append(sys.argv[2])

### maybe perform random seeks
if sys.argv[3] != "0":
    command.append("-random_seeks")
    command.append(sys.argv[3])

### report command string
gp.AddMessage("LAStools command line:")
command_length = len(command)
command_string = str(command[0])
for i in range(1, command_length):
    command_string = command_string + " " + str(command[i])
gp.AddMessage(command_string)

### run command
returncode,output = check_output(command, False)

### report output of lasdiff
gp.AddMessage(str(output))

### check return code
if returncode != 0:
    gp.AddMessage("Error. lasdiff failed.")
    sys.exit(1)

### report happy end
gp.AddMessage("Success. lasdiff done.")
