#!/bin/env python3

import os
import argparse
from glob import glob, escape
import re
import platform
import subprocess

list_files = subprocess.run(["ls", "-l"])


parser = argparse.ArgumentParser(description='LFR coverage report generator')
parser.add_argument('--src-dir')
parser.add_argument('--build-dir')
parser.add_argument('--gcov-output-dir')
parser.add_argument('--gcov-path')
parser.add_argument('--output-dir')
args = parser.parse_args()


build_gcov_files = f"{os.path.dirname(__file__)}/build_gcov_files.py"

gcov_traces = list(filter(lambda f:f.endswith(".txt") ,os.listdir(args.gcov_output_dir)))

def generate_individual_report(gcov_trace):
    output_dir = f"{args.output_dir}/{gcov_trace.replace('.txt','')}"
    os.makedirs(output_dir, exist_ok=True)
    subprocess.run([build_gcov_files, "-r", args.build_dir, "-o", output_dir, f"{args.gcov_output_dir}/{gcov_trace}"])
    subprocess.run(["lcov", "--config-file", f"{os.path.dirname(__file__)}/lcovrc", "--gcov-tool", args.gcov_path, '-c', '-d', output_dir, '-b', args.build_dir ,  '-o', f'{output_dir}/trace.info'])
    subprocess.run(["genhtml", "--config-file", f"{os.path.dirname(__file__)}/lcovrc", '-o', f'{output_dir}',  f'{output_dir}/trace.info'])
    return f'{output_dir}/trace.info'

def generate_total_coverage_report(lcov_traces):
    output_dir = f"{args.output_dir}/total_coverage"
    os.makedirs(output_dir, exist_ok=True)
    lcov_args = ["lcov" ]
    for t in lcov_traces:
        lcov_args+= ['-a',t]
    lcov_args += ["--config-file", f"{os.path.dirname(__file__)}/lcovrc", "-o", f"{output_dir}/trace.info"]
    subprocess.run(lcov_args)
    subprocess.run(["genhtml", "--config-file", f"{os.path.dirname(__file__)}/lcovrc", '-o', f'{output_dir}', f"{output_dir}/trace.info"])

generate_total_coverage_report(list(map(generate_individual_report, gcov_traces)))
