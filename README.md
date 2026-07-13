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

In each sub-directory of this copy of `future`, it runs the Python scripts
(`script.py`) one after another.

The output is written to a file named `benchmark` in the directory
corresponding to the robot's name, next to the script. If a script fails, the
generator stops and the error is available in this file.
