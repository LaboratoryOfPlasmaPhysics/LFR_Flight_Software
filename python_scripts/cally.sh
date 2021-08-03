#!/bin/bash
SRC_DIR=$1
BUILD_DIR=$2

SCRIPT_PATH=$(dirname "$0")

find $BUILD_DIR -name '*.expand' | xargs $SCRIPT_PATH/cally.py  | dot -Grankdir=LR -Tpng -o all_calgraph.png 

TASKS=$(grep -r 'task(' $SRC_DIR | grep -v '.*\.py' | cut -d ' ' -f 2 | sort | uniq | cut -d '(' -f 1)
echo "Tasks: $TASKS"

for task in $TASKS
do 
	find $BUILD_DIR  -name '*.expand' | xargs $SCRIPT_PATH/cally.py --caller "$task"  | dot -Grankdir=LR -Tpng -o "$task"_calgraph.png 
done
