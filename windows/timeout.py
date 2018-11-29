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
