#! /usr/bin/python

import sys, os, re, argparse

parser = argparse.ArgumentParser(prog="createAprilGrid", description="Tool to create AprilTag grids.")
parser.add_argument("-x", "--board-x", help="Number of AprilTags in x direction", type=int, default=0)
parser.add_argument("-y", "--board-y", help="Number of AprilTags in y direction", type=int, default=0)
parser.add_argument("-s", "--pattern-size", help="Size of AprilTag (including the white border)", type=float, default=0)
parser.add_argument("-id", "--start-id", help="First ID of AprilTag grid", type=int, default=0)

args = parser.parse_args()

print("#APRIL_2D")

offset_x = 0
offset_y = 0

tag_size = args.pattern_size / 10.0 * 8.0

for y in range(0, args.board_y):
    for x in range(0, args.board_x):
        print("#" + str(args.start_id + x + args.board_x * y))
        anchor_x = offset_x + x * args.pattern_size
        anchor_y = offset_y + y * args.pattern_size
        print(str(anchor_x) + ' ' + str(anchor_y))
        print(str(anchor_x + tag_size) + ' ' + str(anchor_y))
        print(str(anchor_x + tag_size) + ' ' + str(anchor_y + tag_size))
        print(str(anchor_x) + ' ' + str(anchor_y + tag_size))
