#!/usr/bin/env python

import os, sys
import shutil

def main():
    new_dir_name = sys.argv[1]
    shutil.copytree('future', new_dir_name)
    os.chdir(new_dir_name)
    dir_names = list(filter(os.path.isdir, os.listdir(".")))

    for name in dir_names:
        os.system("xterm -e hppcorbaserver | xterm -e ipython " + name + "/script.py")

if __name__ == "__main__":
    main()