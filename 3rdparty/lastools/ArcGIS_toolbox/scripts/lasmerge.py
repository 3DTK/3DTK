#
# lasmerge.py
#
# (c) 2012, Martin Isenburg
# LASSO - rapid tools to catch reality
#
# uses lasmerge.exe to merge several LiDAR files into one.
#
# The LiDAR input can be in LAS/LAZ/BIN/TXT/SHP/ASC... format.
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
gp.AddMessage("Starting lasmerge ...")

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

### create the full path to the lasmerge executable
lasmerge_path = lastools_path+"\\lasmerge.exe"

### check if executable exists
if os.path.exists(lastools_path) == False:
    gp.AddMessage("Cannot find lasmerge.exe at " + lasmerge_path)
    sys.exit(1)
else:
    gp.AddMessage("Found " + lasmerge_path + " ...")

### create the command string for lasmerge.exe
command = [lasmerge_path]

### maybe use '-verbose' option
if sys.argv[argc-1] == "true":
    command.append("-v")

### add first and second input LiDAR
command.append("-i")
command.append(sys.argv[1])
command.append(sys.argv[2])
   
### maybe add another input LiDAR
if sys.argv[3] != "#":
    command.append(sys.argv[3])

### maybe add another input LiDAR
if sys.argv[4] != "#":
    command.append(sys.argv[4])

### maybe add another input LiDAR
if sys.argv[5] != "#":
    command.append(sys.argv[5])

### maybe add another input LiDAR
if sys.argv[6] != "#":
    command.append(sys.argv[6])

### maybe add another input LiDAR
if sys.argv[7] != "#":
    command.append(sys.argv[7])

### maybe add another input LiDAR
if sys.argv[8] != "#":
    command.append(sys.argv[8])

### maybe add another input LiDAR
if sys.argv[9] != "#":
    command.append(sys.argv[9])

### add output LiDAR
command.append("-o")
command.append(sys.argv[10])

### report command string
gp.AddMessage("LAStools command line:")
command_length = len(command)
command_string = str(command[0])
for i in range(1, command_length):
    command_string = command_string + " " + str(command[i])
gp.AddMessage(command_string)

### run command
returncode,output = check_output(command, False)

### report output of lasmerge
gp.AddMessage(str(output))

### check return code
if returncode != 0:
    gp.AddMessage("Error. lasmerge failed.")
    sys.exit(1)

### report happy end
gp.AddMessage("Success. lasmerge done.")
