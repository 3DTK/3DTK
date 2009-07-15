#!/usr/bin/env python
#
# converts all .frames files in a directory into the new format

import os, sys

def main():
  if len(sys.argv) != 2:
    print 'converts all .frames files in a directory into the new format'
    print 'run with %s directory' % sys.argv[0]
    return
  directory = sys.argv[1]
  for name in os.listdir(directory):
    fullpath = os.path.join(directory, name)
    if os.path.isfile(fullpath) and fullpath.endswith('.frames'):
      frame = file(fullpath).read()
      frame = frame.replace('\n\n', '\n')
      frame = frame.replace('\n-1 0 0 1\n', '0\n')
      frame = frame.replace('\n-1 1 0 1\n', '0\n')
      frame = frame.replace('\n0 0 1 1\n', '1\n')
      frame = frame.replace('\n1 1 0 1\n', '2\n')
      frame = frame.replace('\n1 0 0 1\n', '3\n')
      file(fullpath, 'w').write(frame)

if __name__ == '__main__':
  main()
