#!/usr/bin/env python

import os, sys
from subprocess import Popen
import shutil

script_name = "script.py"
benchmark_descr_dir = "future" # directory with description of all benchmarks
benchmark_file = "benchmark"
def main():
    new_dir_name = sys.argv[1]
    shutil.copytree(benchmark_descr_dir, new_dir_name)
    os.chdir(new_dir_name)
    # note that "pr2-in-iai-kitchen" and "pyrene-on-the-ground" do not use
    # the manipulation plugin ("manipulation-corba.so") so we should either
    # restart hppcorbaserver each time, or to run these two benchmarks before
    # running the rest which loads the plugin

    # get names of all immediate subdirectories that contain a script.py file
    dir_names = [f.name for f in os.scandir(".") if f.is_dir()
                and os.path.isfile(os.path.join(f.path, script_name))]

    for name in dir_names:
        # reset the server for every new benchmark
        server_proc = Popen(["hppcorbaserver"])
        print(f"Executing {name}/script.py")
        with open(f"{name}/benchmark", "w") as log_file:
            client_proc = Popen(['python3', f'{name}/script.py'],
                                stdout=log_file, stderr=log_file)
            client_proc.wait()
        server_proc.terminate()
        server_proc.wait()

if __name__ == '__main__':
    if len(sys.argv) != 2:
        raise RuntimeError("Expected 1 argument: new directory name")
    main()
