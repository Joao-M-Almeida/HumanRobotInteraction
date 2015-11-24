#!/usr/bin/env python
import glob, os
os.chdir("TrainingData")
for file in glob.glob("*.txt"):
    newlines=[]
    print(file)
    f = open(file, "r+")
    lines = f.readlines()
    print(lines)
    for line in lines:
        line = line.replace(" ", "")
        line = line.replace(",", " ")
        line = line.replace("[", "")
        line = line.replace("]", "")
        newlines.append(line)
    print(newlines)
    f.seek(0)
    for newline in newlines:
        f.write(newline)
    f.close()
