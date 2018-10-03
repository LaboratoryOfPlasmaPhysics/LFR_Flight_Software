#!/usr/bin/env python3

__author__ = "Alexis Jeandet"
__copyright__ = "Copyright 2018, Laboratory of Plasma Physics"
__credits__ = []
__license__ = "GPLv2"
__version__ = "1.0.0"
__maintainer__ = "Alexis Jeandet"
__email__ = "alexis.jeandet@member.fsf.org"
__status__ = "Development"

import argparse
import subprocess



parser = argparse.ArgumentParser()
parser.add_argument("-s", "--sources", help="Source path path", required=True)
parser.add_argument("-o", "--output-folder", help="Will generate html report into this folder", required=True)
parser.add_argument("-g", "--gcov-exe", help="Gcov executable", required=True)
parser.add_argument("path", help="Path where are located gcda and gcno files")

args = parser.parse_args()

def main():
    p = subprocess.Popen(["gcovr",
                          "--gcov-executable=" + args.gcov_exe,
                          "--object-directory=" + args.path,
                          "-r=" + args.sources,
                          "--html",
                          "--html-details",
                          "-o=" + args.output_folder + "/gcov.html"
                          ],
                         stdout=subprocess.PIPE)


if __name__ == "__main__":
    main()
