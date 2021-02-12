#!/usr/bin/env python
from __future__ import print_function
import os, sys
from generate_benchmark_plot import readCSV, generateErrorBars

# To generate this list:
# find 20*/* -type d | sed 's/^[^\/]*\///' | sort | uniq

root_dir="."
result_dir="./results/cpp"
script_dir="./script"
try:
    output_dir = sys.argv[1]
except:
    output_dir="./results/cpp"

if not os.path.isdir(output_dir): os.mkdir(output_dir)

csv_files = []
n = len(result_dir) + 1
for root, dirs, files in os.walk (result_dir):
    for f in filter(lambda fn: fn.endswith(".csv"), files):
        csv_files.append(os.path.join(root[n:], f[:-4]))

csv_files.sort()

benchmarks = dict()
for csv in csv_files:
    year, month, day, bench = csv.split(os.path.sep, 3)
    if bench not in benchmarks:
        benchmarks[bench] = list()
    benchmarks[bench].append(csv)

for bench, csvfiles in benchmarks.items():
    legends, labels, rows = [], [], []
    for csv in csvfiles:
        le, la, ro = readCSV(os.path.join(result_dir,csv+".csv"))
        # TODO check that all element in labels are the same ?
        #for r in ro.T:
            #rows.append(r)
            #labels.append(la[0])
        if len(legends) == 0:
            legends = le
        else:
            assert legends == le
        rows.append(ro)
        labels.append(la[0])
    if len(rows) == 0:
        print("This should not happen: No data in file " + csv)
        continue

    svg = os.path.join (output_dir, bench.replace('/','_') + ".svg")
    generateErrorBars (legends, labels, rows, bench, svg)
