#!/usr/bin/env python

import os, sys
import shutil

script_name = "script.py"
benchmark_descr_dir = "future" # directory with description of all benchmarks
benchmark_file = "benchmark"
def main():
    new_dir_name = sys.argv[1]
    shutil.copytree(benchmark_descr_dir, new_dir_name)
    os.chdir(new_dir_name)
    # get names of all immediate subdirectories that contain a script.py file
    dir_names = [f.name for f in os.scandir(".") if f.is_dir()
                and os.path.isfile(os.path.join(f.path, script_name))]

    os.system("xterm -e hppcorbaserver &")
    for name in dir_names:
        print(f"Executing {name}/script.py")
        os.system(f"xterm -e 'python3 {name}/script.py > {name}/benchmark 2>&1'")

if __name__ == '__main__':
    if len(sys.argv) != 2:
        raise RuntimeError("Expected 1 argument: new directory name")
    main()