import subprocess
import sys

p = subprocess.Popen(sys.argv[2:])
try:
    p.wait(timeout=int(sys.argv[1]))
except subprocess.TimeoutExpired as e:
    # there is no psutil module installed on appveyor
    subprocess.call(['taskkill', '/F', '/T', '/PID', str(p.pid)])
    exit(1)
