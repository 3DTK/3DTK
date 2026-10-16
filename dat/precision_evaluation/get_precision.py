import pathlib
import sys
import os
import argparse

import numpy as np
import math as m


# Describe functionality
# def usage():
#     print("Usage:")
#     print()
#     print("Examples:")
#     print("python3 get_precision.py -d <directory>")
#     print("python3 get_precision.py -s <start number> -e <end number> -d <directory>")
#     print()
#     print("-s \t number of first scan to be considered")
#     print("-e \t number of last scan to be considered")
#     print("-d \t directory of scans to be considered")

# # Print error message and leave program
# def abort(errorMessage):
#     print("\033[91m" + errorMessage + "\033[0m")
#     print()
#     usage()
#     sys.exit(1)


    #ÄNDERN DASS SINNVOLL:
    #- Transformationsmatrix bleibt
    # - pose file ändern, scan kopieren und umbenennen
    # random änderungen in pose file -> matrix erstellen -> icp aufrufen -> werte vergleichen
    # -> mitteln? oder in tatsächliche werte: grad und cm/mm umrechnen
    # -> darstellen

    ## UND ACHTUNG WEGEN x und z IN VEKTOR VERTAUSCHT

# Transform model scan in acc. to entered translation and rotation vector
def main():

    parser = argparse.ArgumentParser( prog = 'get_precision', description = 'Get precision of given frame number')
    parser.add_argument("-d", "--directory", required = True, help="directory of scans to be considered")
    parser.add_argument("-s", "--start", type = int, help="start number")
    parser.add_argument("-e", "--end", type = int, help="end number")

    args = parser.parse_args()
    dir = args.directory
    start = args.start
    end = args.end

   # if not os.path.exists(dir):
    #    parser.error("Path does not exist")

    path = pathlib.Path(dir)
    if not path.is_dir():
        parser.error("Path is not a directory")

    i = 0
    pose = path/f"scan{i:03d}.pose"
    frame = path/f"scan{i:03d}.frames"

    # Open directory and start with first pose / frame
    while frame.exists() & pose.exists():
        with open(pose) as p:
            lines = p.readlines()
            # rotation and translation vector
            tvec = [float(t) for t in lines[0].split()]
            rvec = [float(r) for r in lines[1].split()]

            # Generate rotation matrix from euler angles
            x = m.radians(rvec[0])  # phi
            y = m.radians(rvec[1])  # theta
            z = m.radians(rvec[2])  # psi
            R = np.array([[m.cos(y) * m.cos(z), -m.cos(y) * m.sin(z), m.sin(y)],
                          [m.cos(z) * m.sin(x) * m.sin(y) + m.cos(x) * m.sin(z),
                           m.cos(x) * m.cos(z) - m.sin(x) * m.sin(y) * m.sin(z), -m.cos(y) * m.sin(x)],
                          [m.sin(x) * m.sin(z) - m.cos(x) * m.cos(z) * m.sin(y),
                           m.cos(z) * m.sin(x) + m.cos(x) * m.sin(y) * m.sin(z), m.cos(x) * m.cos(y)]]
                         )

        pose = path / f"scan{i:03d}.pose"
        frame = path / f"scan{i:03d}.frame"
        i += 1

    # # initiate rotation/translation vector





            with open(dir + "/scan" + number_string + "_real_transformation.txt", "w", encoding="utf-8") as transformation:
                # EVTL WIEDER WEG UND DATEN GENAU IN FORM DER FRAMES DATEI DASS DIREKT VERGLEICHBAR
                transformation.write("Translation" + "\n")
                transformation.write(str(tvec[0]) + "\t" + str(tvec[1]) + "\t" + str(tvec[2]) + "\n")
                transformation.write("Rotation" + "\n")
                transformation.write(str(rvec[0]) + "\t" + str(rvec[1]) + "\t" + str(rvec[2]) + "\n")
                transformation.write("Transformation matrix" + "\n")
                for arr in R:
                    transformation.write(str(arr[0]) + " " + str(arr[1]) + " " + str(arr[2]) + " "+ "0" + " ")
                transformation.write(str(tvec[0]) + " " + str(tvec[1]) + " " + str(tvec[2]) + " "+ "1")

            with open(dir,encoding="utf-8") as scan:
                for line in scan:
                    d_str = line.split();
                    d = np.array([0.0, 0.0, 0.0])
                    for i in range(len(d_str)):
                        try:
                            d[i] = float(d_str[i])
                        except ValueError:
                            abort("Invalid data in model scan")
            # Calculate transformed/rotated points d' = Rd + t
                    d_new = R @ d + tvec
                    new_file.write(str(d_new[0]) + "\t" + str(d_new[1]) + "\t" + str(d_new[2]) + "\n")

if __name__ == "__main__":
    main()