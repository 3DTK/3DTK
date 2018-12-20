#!/usr/bin/env python3

# Run the build with our own timeout because the vcpkg cache is not saved if
# we run into the appveyor timeout of one hour. Thus we abort after 80
# minutes (4800 seconds). Ten minutes are left for the initial git clone plus
# downloading/extracting/zipping/uploading of the cache.
#
# We use python to implement the process timeout - have fun trying to find
# how to do it using Windows tools like cmd.exe or the powershell.
#
# With cmd.exe, the script is executed inside the same process and thus you
# cannot kill it after a timeout.
#
# With powershell, Start-Job does not start the script in a way that directly
# redirects its output to standard output but only allows to collect it in the
# end.

import subprocess
import sys

p = subprocess.Popen(sys.argv[2:])
try:
    exit(p.wait(timeout=int(sys.argv[1])))
except subprocess.TimeoutExpired as e:
    # there is no psutil module installed on appveyor
    subprocess.call(['taskkill', '/F', '/T', '/PID', str(p.pid)])
    # appveyor seems to throw the cache away if the build exits with a non-zero
    # exit code even if APPVEYOR_SAVE_CACHE_ON_ERROR is set to "true"
    exit(0)
