#!/usr/bin/env python

import sys

values = list()

allrefl = list()

with open(sys.argv[1]) as f:
    for line in f:
        tokens = line.split()
        refl = float(tokens[-1])
        allrefl.append(refl)
        values.append(tokens[:-1]+[refl])

allrefl.sort()

minrefl = allrefl[int(len(allrefl)*0.05)]
maxrefl = allrefl[-int(len(allrefl)*0.05)]

with open(sys.argv[1], "w") as f:
    for tokens in values:
        tokens[-1] = (tokens[-1] - minrefl)/(maxrefl-minrefl)
        f.write(" ".join([str(t) for t in tokens])+"\n")
