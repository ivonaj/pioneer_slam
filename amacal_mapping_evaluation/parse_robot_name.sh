#!/bin/bash

if [ "$#" -ne 3 ]; then
    echo "Usage: $0 robot-name in-file out-file"
    echo "Replaces occurences of \${ROBOT_NAME} in in-file. The result is written to out-file."
    exit 1
fi

perl -pe 's/([^\\]|^)\$\{ROBOT_NAME\}/$1.'"$1"'/eg' "$2" | install -D /dev/stdin $3
