# hpp_benchmark

A set of benchmark to track the evolution of performances of HPP.

## Running the C++ benchmarks

Compile target `benchmarks` and run
```cpp
benchmarks --output {source_dir}/results/cpp/{year}/{month}/{day} --label <version>
```
Option `--output <dir>` can be omitted if you do not want to save the results to the disk.
See `benchmarks --help` for more usage.

### Save the results
The script will generate a bunch of csv files in the output directory (see `--output`).
To save the benchmarks, commit the output directory.

## Running the Python benchmarks
```
 ./generate_benchmark.py  new_directory_name
```

The format of the argument `new_directory_name` is: `year-month-day`
This program creates a copy of the directory `future` into a new directory which contains the benchmarks.

In each sub-directories of this copy of future, it will run one after another the python scripts (script.py). For each script 2 windows will be opened: a window with `hppcorbaserver` and another window where the `script.py` is running. So a script is running only if these two windows are opended.

When the execution of a script is done, the python's window is closed and the output will be written in a file named `benchmark` in the directory corresponding to the robot's name, next to the script. The `hppcorbaserver` window still open.

To execute the next script you have to close this window. Two new windows automatically appear (`hppcorbaserver`and `python`) which means that a new script is running.
