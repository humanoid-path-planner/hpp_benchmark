#!/usr/bin/env python
from __future__ import print_function
import sys
from os.path import isdir, join, isfile
from os import listdir
import typing as T

benchmark_file = "benchmark"
def parseDirectories(directory:str, bench:str) -> T.Tuple[T.List[str], T.Dict[str, T.Dict[str, float]]]:
    """Parse a directory to get average statistics from all benchmark files
    of a specified type

    The average statistics must start with "Average "

    Args:
        directory: path to directory containing the benchmark files
        bench: type of the benchmark, eg "baxter-manipulation-boxes"

    Returns:
        keys: list of names of the statistics
        values: dictionary mapping subdirectory name to a dictionary
          of (statistics, value)
    """
    keys = []
    values = dict()

    years = list(filter(lambda s:s.isdigit(), listdir(directory)))
    for y in years:
        days = list(filter(lambda s:s[0:2].isdigit() and s[2]=='-' and s[3:].isdigit(),
                           listdir(join(directory, y))))
        for d in days:
            print (f"year={y}, day={d}")
            file = join(directory, y, d, bench, benchmark_file)
            dir = y + "/" + d
            if not isfile(file):
                print(file + " does not exist", file=sys.stderr)
                continue
            values[dir] = dict()
            with open (file, 'r') as f:
                for line in f.readlines():
                    if line.startswith("Average "):
                        k, v = line[len("Average "):].strip().split(':', 1)
                        if k not in keys: keys.append(k)
                        values[dir][k] = float(v)
    return keys, values

def generateCSV(keys: T.List[str], values: T.Dict[str, T.Dict[str, float]], csvfile: str, sep: str = ';', nan: str = 'nan'):
    labels = sorted(values.keys())
    print("Dates" + sep + sep.join(keys), file=csvfile)
    for d in labels:
        line = d
        val = values[d]
        # print(val)
        for k in keys:
            if k in val.keys():
                line += sep + str(val[k])
            else:
                line += sep + nan
        print(line, file=csvfile)

if __name__ == '__main__':
    if len(sys.argv) != 3:
        raise RuntimeError("Expected 1 argument")

    directory = sys.argv[1]
    bench = sys.argv[2]

    keys, values = parseDirectories (directory, bench)
    generateCSV (keys, values, sys.stdout)
