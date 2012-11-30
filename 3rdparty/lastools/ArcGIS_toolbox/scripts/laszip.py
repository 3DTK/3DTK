#
# laszip.py
#
# (c) 2012, Martin Isenburg
# LASSO - rapid tools to catch reality
#
# uses laszip.exe to thinning LiDAR points by placing a uniform grid over
# them and keeping only the point with the lowest (or 'highest' or 'random')
# Z coordinate within each grid cell.
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
gp.AddMessage("Starting laszip ...")

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

### create the full path to the laszip executable
laszip_path = lastools_path+"\\laszip.exe"

### check if executable exists
if os.path.exists(lastools_path) == False:
    gp.AddMessage("Cannot find laszip.exe at " + laszip_path)
    sys.exit(1)
else:
    gp.AddMessage("Found " + laszip_path + " ...")

### create the command string for laszip.exe
command = [laszip_path]

### maybe use '-verbose' option
if sys.argv[argc-1] == "true":
    command.append("-v")

### add input LiDAR
command.append("-i")
command.append(sys.argv[1])

### maybe only report size
if sys.argv[2] == "true":
    command.append("-size")
        
### maybe also compress/decompress waveforms
if sys.argv[3] == "true":
    command.append("-waveforms")
    command.append(sys.argv[2])
        
### maybe an output format was selected
if sys.argv[4] != "#":
    if sys.argv[4] == "las":
        command.append("-olas")
    elif sys.argv[4] == "laz":
        command.append("-olaz")
    elif sys.argv[4] == "bin":
        command.append("-obin")
    elif sys.argv[4] == "xyz":
        command.append("-otxt")
    elif sys.argv[4] == "xyzi":
        command.append("-otxt")
        command.append("-oparse")
        command.append("xyzi")
    elif sys.argv[4] == "txyzi":
        command.append("-otxt")
        command.append("-oparse")
        command.append("txyzi")

### maybe an output file name was selected
if sys.argv[5] != "#":
    command.append("-o")
    command.append(sys.argv[5])

### maybe an output directory was selected
if sys.argv[6] != "#":
    command.append("-odir")
    command.append(sys.argv[6])

### maybe an output appendix was selected
if sys.argv[7] != "#":
    command.append("-odix")
    command.append(sys.argv[7])

### report command string
gp.AddMessage("LAStools command line:")
command_length = len(command)
command_string = str(command[0])
for i in range(1, command_length):
    command_string = command_string + " " + str(command[i])
gp.AddMessage(command_string)

### run command
returncode,output = check_output(command, False)

### report output of laszip
gp.AddMessage(str(output))

### check return code
if returncode != 0:
    gp.AddMessage("Error. laszip failed.")
    sys.exit(1)

### report happy end
gp.AddMessage("Success. laszip done.")
