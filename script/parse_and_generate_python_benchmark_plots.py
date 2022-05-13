#!/usr/bin/env python
import os, sys
from parse_benchmark import parseDirectories, generateCSV
from generate_benchmark_plot import readCSV, generatePlot

# To generate this list:
# find 20*/* -type d | sed 's/^[^\/]*\///' | sort | uniq
if len(sys.argv) > 2:
    benchmarks = sys.argv[2:]
else:
    script_name = "script.py"
    benchmark_descr_dir = "./future" # folder that describe the benchmarks
    # get names of all immediate subdirectories of "future/" that contain a script.py file
    benchmarks = [f.name for f in os.scandir(benchmark_descr_dir) if f.is_dir()
                and os.path.isfile(os.path.join(f.path, script_name))]

root_dir="."
script_dir="./script"
try:
    output_dir = sys.argv[1]
except:
    output_dir="./results"

if not os.path.isdir(output_dir): os.mkdir(output_dir)

for bench in benchmarks:
    csv = os.path.join (output_dir, bench + ".csv")

    keys, values = parseDirectories (root_dir, bench)

    if len(keys) == 0:
        print("No data for bench " + bench)
        continue

    with open(csv, 'w') as csvfile:
        generateCSV (keys, values, csvfile)

    legends, labels, rows = readCSV(csv)
    if len(rows) == 0:
        print("This should not happen: No data in file " + csv)
        continue
    svg = os.path.join (output_dir, bench + ".svg")
    generatePlot (legends, labels, rows, bench, svg)
