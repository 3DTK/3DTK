#
# lasprecision.py
#
# (c) 2012, Martin Isenburg
# LASSO - rapid tools to catch reality
#
# uses lasprecision.exe to compute statistics indicating if the precision
# "advertised" is really in the data. Often the scaling factors of a LAS
# file are miss-leading because they make it appear as if there was much
# more precision than there really is.
#
# The LiDAR input can be in LAS/LAZ/BIN/TXT/SHP/... format.
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
gp.AddMessage("Starting lasprecision ...")

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

### create the full path to the lasprecision executable
lasprecision_path = lastools_path+"\\lasprecision.exe"

### check if executable exists
if os.path.exists(lastools_path) == False:
    gp.AddMessage("Cannot find lasprecision.exe at " + lasprecision_path)
    sys.exit(1)
else:
    gp.AddMessage("Found " + lasprecision_path + " ...")

### create the command string for lasprecision.exe
command = [lasprecision_path]

### maybe use '-verbose' option
if sys.argv[argc-1] == "true":
    command.append("-v")

### add input LiDAR
command.append("-i")
command.append(sys.argv[1])

### maybe a user-specified rescaling
if sys.argv[2] == "true":
    if (sys.argv[3] != "#") and (sys.argv[4] != "#") and (sys.argv[5] != "#"):
        command.append("-rescale")
        command.append(sys.argv[3])
        command.append(sys.argv[4])
        command.append(sys.argv[5])
        
### maybe an output format was selected
if sys.argv[5] != "#":
    if sys.argv[5] == "las":
        command.append("-olas")
    elif sys.argv[5] == "laz":
        command.append("-olaz")
    elif sys.argv[5] == "bin":
        command.append("-obin")
    elif sys.argv[5] == "xyz":
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

### report output of lasprecision
gp.AddMessage(str(output))

### check return code
if returncode != 0:
    gp.AddMessage("Error. lasprecision failed.")
    sys.exit(1)

### report happy end
gp.AddMessage("Success. lasprecision done.")
