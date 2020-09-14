#!/usr/bin/env python

import os, sys
import shutil

def main():
    new_dir_name = sys.argv[1]
    shutil.copytree('future', new_dir_name)
    os.chdir(new_dir_name)
    dir_names = ["pr2-in-iai-kitchen","hrp2-on-the-ground",  "baxter-manipulation-boxes", "construction-set", "pr2-manipulation-kitchen", "pr2-manipulation-two-hand",
    "romeo-placard", "ur3-spheres"]

    os.system("xterm -e hppcorbaserver &")
    for name in dir_names:
        print("Executing " + name + "/script.py")
        os.system("xterm -e 'python3 " + name + "/script.py > " + name + "/benchmark 2>&1'")

if __name__ == '__main__':
    main()